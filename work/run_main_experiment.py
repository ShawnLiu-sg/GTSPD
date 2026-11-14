import argparse
import csv
import ctypes
import json
import os
from pathlib import Path
from queue import Queue
from statistics import mean
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed


work = Path(__file__).resolve().parent
cases_root = work / 'formal_cases'
compare = work / 'compare-build' / 'truck_drone_compare.exe'
solver = work / 'solver-build' / 'truck_drone.exe'
methods = ('hybrid', 'plain', 'fixed_q', 'nonadaptive', 'within_stop', 'order_first', 'vns')
seeds = tuple(range(101,111))
full_budgets = {50:5000,100:15000}
screen_budgets = {50:350,100:800}


def pinned_run(cpu, command):
    # Set the launcher thread's process affinity before creating the child, then restore it.
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    set_thread_mask = kernel.SetThreadAffinityMask
    set_thread_mask.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
    set_thread_mask.restype = ctypes.c_size_t
    kernel.GetCurrentThread.restype = ctypes.c_void_p
    current = kernel.GetCurrentThread()
    previous = set_thread_mask(current, 1 << cpu)
    if not previous:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        proc = subprocess.Popen([str(x) for x in command], stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, errors='replace')
        # The child starts single-threaded and stays on the assigned logical CPU.
        set_process_mask = kernel.SetProcessAffinityMask
        set_process_mask.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
        if not set_process_mask(int(proc._handle), 1 << cpu) and proc.poll() is None:
            proc.kill()
            raise ctypes.WinError(ctypes.get_last_error())
        stdout, stderr = proc.communicate()
        if proc.returncode:
            raise RuntimeError(f'{command} exited {proc.returncode}: {stderr}')
        return stdout
    finally:
        set_thread_mask(current, previous)


def jobs(cpus, items, callback):
    available = Queue()
    for cpu in cpus:
        available.put(cpu)

    def on_cpu(item):
        cpu = available.get()
        try:
            return callback(cpu,item)
        finally:
            available.put(cpu)

    with ThreadPoolExecutor(max_workers=len(cpus)) as executor:
        futures = [executor.submit(on_cpu, item) for item in items]
        for future in as_completed(futures):
            yield future.result()


def records(path):
    with path.open(newline='', encoding='utf-8') as source:
        return list(csv.DictReader(source))


def manifest(case, path, budget, seed_list, method_list, settings, target, rotate=0):
    checkpoints = [budget//10,budget//4,budget//2,budget]
    method_list = method_list[rotate:] + method_list[:rotate]
    text = '\n'.join([
        'BENCHMARK 1',f'BUDGET_MS {budget}',
        'CHECKPOINTS 4 '+' '.join(map(str,checkpoints)),
        f'SEEDS {len(seed_list)} '+' '.join(map(str,seed_list)),
        f'METHODS {len(method_list)} '+' '.join(method_list),f'FIXED_Q {settings["fixed_q"]}',
        f'ORDER_FIRST {settings["block"]} {settings["perturb"]}', f'VNS {settings["shake"]}',
        'BOOTSTRAP 9999 153072', 'CASES 1',
        f'{case["case"]} n{case["n"]}_{case["geometry"]} "{path.as_posix()}" {target}', 'END', ''
    ])
    return text


def launch(cpu, result, config):
    result.mkdir(parents=True, exist_ok=True)
    input_file = result / 'manifest.txt'
    input_file.write_text(config, encoding='ascii')
    message = pinned_run(cpu,[compare,input_file,result])
    if not (result / 'runs.csv').exists():
        raise RuntimeError(message)
    (result / 'complete.txt').write_text(f'cpu={cpu}\n{message}', encoding='utf-8')
    return records(result / 'runs.csv')


def tune(cpus, root, data):
    config_path = root / 'frozen_tuning.json'
    if config_path.exists():
        return json.loads(config_path.read_text(encoding='utf-8'))
    choices = {
        'fixed_q': ('fixed_q', {50:[3,5,8],100:[4,8,14]}),
        'block': ('order_first',{50:[4,5,6],100:[4,5,6]}),
        'shake': ('vns',{50:[1,2,3],100:[1,2,3]})
    }
    defaults = {'fixed_q':4,'block':5,'perturb':2,'shake':2}
    items = [(case,key,method,value) for case in data if case['group']=='tuning'
             for key,(method,grid) in choices.items() for value in grid[int(case['n'])]]

    def run(cpu,item):
        case,key,method,value = item
        path = root / 'tuning' / f'{key}_{value}' / case['case']
        settings = defaults | {key:value}
        result = records(path/'runs.csv') if (path/'complete.txt').exists() else launch(cpu,path,
            manifest(case,cases_root/case['path'],screen_budgets[int(case['n'])],(81,82),(method,),settings,'-'))
        vals = [float(row['objective'])/float(row['initial_objective']) for row in result if row['status']=='ok']
        return case['n'], key, value, mean(vals), len(vals)

    outcomes = list(jobs(cpus, items, run))
    settings_by_size = {str(n):dict(defaults) for n in full_budgets}
    ranking = []
    for n, frozen in settings_by_size.items():
        for key,(_,grid) in choices.items():
            options = []
            for value in grid[int(n)]:
                runs = [x for x in outcomes if x[:3]==(n,key,value)]
                option = (-sum(x[4] for x in runs), mean(x[3] for x in runs), value)
                options.append(option)
                ranking.append({'n':int(n),'parameter':key,'value':value,'valid_runs':-option[0],
                                'mean_initial_relative_cost':option[1]})
            frozen[key] = min(options)[2]
    payload = {'settings':settings_by_size,'rankings':ranking,'tuning_caps_ms':screen_budgets,
               'tuning_seeds':[81,82],'fixed_other_alns_parameters':'instance file'}
    config_path.write_text(json.dumps(payload,indent=2), encoding='utf-8')
    print('Frozen tuning:',json.dumps(settings_by_size),flush=True)
    return payload


def targets(cpus, root, main):
    path = root / 'frozen_targets.json'
    if path.exists():
        return json.loads(path.read_text(encoding='utf-8'))
    target_inputs = root / 'target_inputs'
    target_inputs.mkdir(exist_ok=True)

    def calculate(cpu,case):
        base = (cases_root/case['path']).read_text(encoding='ascii')
        # Freeze an independently specified 5% improvement over the all-truck NN + 2-opt plan.
        inp = target_inputs / f'{case["case"]}.txt'
        inp.write_text(base.replace('SEARCH 101 1000000 15000','SEARCH 101 0 5000'), encoding='ascii')
        value = json.loads(pinned_run(cpu,[solver,inp]))
        if value['iterations'] != 0 or value['sorties']:
            raise RuntimeError('Target reference must be an all-truck construction')
        return case['case'], {'truck_reference':value['objective'],'target':0.95*value['objective']}

    results = dict(jobs(cpus,main,calculate))
    path.write_text(json.dumps(results,indent=2,sort_keys=True), encoding='utf-8')
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', choices=('screen','formal'))
    parser.add_argument('--cpus',nargs='+',type=int, default=[0,2,4,6])
    parser.add_argument('--reverse',action='store_true')
    parser.add_argument('--limit',type=int)
    parser.add_argument('--offset',type=int,default=0)
    args = parser.parse_args()
    root = work / 'formal_main'
    root.mkdir(exist_ok=True)
    all_cases = records(cases_root/'instances.csv')
    tested = [case for case in all_cases if case['group']=='main']
    frozen = tune(args.cpus,root,all_cases)
    target_values = targets(args.cpus,root,tested)
    budgets = full_budgets if args.stage=='formal' else screen_budgets
    seed_list = seeds if args.stage=='formal' else seeds[:3]

    def run(cpu,entry):
        i,case = entry
        name = case['case']
        out = root / args.stage / 'results' / name
        if (out/'complete.txt').exists():
            return name,0
        result = launch(cpu,out,manifest(case,cases_root/case['path'],budgets[int(case['n'])],seed_list,
            methods,frozen['settings'][case['n']],format(target_values[name]['target'],'.15g'),i%len(methods)))
        if len(result) != len(seed_list)*len(methods):
            raise RuntimeError(f'Invalid run count for {name}')
        return name,len(result)

    # Interleave geometries and scale to avoid running a full stratum at the same point in time.
    tested.sort(key=lambda case:(int(case['replica']),case['geometry'],int(case['n'])))
    entries = list(enumerate(tested))
    if args.reverse:
        entries.reverse()
    entries = entries[args.offset:]
    if args.limit is not None:
        entries = entries[:args.limit]
    for done,(name,count) in enumerate(jobs(args.cpus,entries,run),1):
        print(f'{args.stage} {done}/{len(tested)} {name} {count}',flush=True)


if __name__ == '__main__':
    main()
