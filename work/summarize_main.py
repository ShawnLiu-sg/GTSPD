import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import random
from statistics import mean, stdev

from verify_compare import decode_sorties, independent_evaluation


workspace = Path(__file__).resolve().parents[1]
work = workspace / 'work'
main = work / 'formal_main'
out = workspace / 'outputs' / 'experiment-results'
method_order = ('hybrid', 'plain', 'fixed_q', 'nonadaptive', 'within_stop', 'order_first', 'vns')


def read(path):
    with path.open(newline='', encoding='utf-8-sig') as handle:
        return list(csv.DictReader(handle))


def write(path, rows):
    if not rows:
        return
    with path.open('w', newline='', encoding='utf-8-sig') as handle:
        writer = csv.DictWriter(handle, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def interval(groups, rng, count=9999):
    if not groups or any(len(group) < 2 for group in groups):
        return '', ''
    n = sum(map(len, groups))
    draws = []
    for _ in range(count):
        draws.append(sum(sum(rng.choices(group, k=len(group))) for group in groups) / n)
    draws.sort()
    return draws[int(.025*(count-1))], draws[int(.975*(count-1))]


def observed(rows, key):
    return [float(row[key]) for row in rows if row[key] != '']


def aggregate(stage):
    destination = out / ('algorithm_formal' if stage == 'formal' else 'algorithm_screening')
    destination.mkdir(parents=True, exist_ok=True)
    results = main / stage / 'results'
    complete = sorted(p.parent for p in results.glob('*/complete.txt'))
    runs, cps, table = [], [], []
    bad_runs = []
    for path in complete:
        cpu = int((path/'complete.txt').read_text(encoding='utf-8').splitlines()[0].split('=')[1])
        instance = (work/'formal_cases'/'cases'/f'{path.name}.txt').read_text(encoding='ascii')
        for row in read(path/'runs.csv'):
            row['logical_cpu'] = cpu
            row['independent_check'] = 'no_incumbent'
            if row['status'] == 'ok':
                tour = list(map(int,row['truck_tour'].split()))
                sorties = decode_sorties(row['sorties'])
                reference = independent_evaluation(instance,tour,sorties)
                if any(not math.isclose(float(row[key]), value, rel_tol=1e-10, abs_tol=1e-9)
                       for key,value in reference.items()):
                    bad_runs.append((row['case'],row['method'],row['seed']))
                else:
                    row['independent_check'] = 'verified'
            runs.append(row)
        cps.extend(read(path/'checkpoints.csv'))
        table.extend(read(path/'table7.csv'))

    if not runs:
        return {'stage':stage,'cases':0,'expected_cases':120}
    assert not bad_runs,bad_runs
    expected = (70 if stage=='formal' else 21) * len(complete)
    assert len(runs) == expected and len(cps) == 4*expected and len(table) == 7*len(complete)
    keyed = {(r['case'],r['seed'],r['method']):r for r in runs}
    assert len(keyed) == expected
    for row in cps:
        last = keyed[row['case'],row['seed'],row['method']]
        if row['checkpoint_ms'] == last['budget_ms']:
            assert row['objective'] == last['objective']
    write(destination/'runs.csv',runs)
    write(destination/'checkpoints.csv',cps)
    write(destination/'table7_cases.csv',table)

    grouped = defaultdict(list)
    for row in runs:
        n = int(row['case'].split('_')[0][1:])
        grouped[(n,row['stratum'],row['method'])].append(row)
        grouped[(n,'all',row['method'])].append(row)
    performance = []
    for (n,stratum,method), block in sorted(grouped.items()):
        valid = [r for r in block if r['status']=='ok']
        per_case = defaultdict(list)
        for row in valid:
            per_case[row['case']].append(float(row['objective']))
        vals = [mean(values) for values in per_case.values()]
        censored = sum(r['target_status']=='right_censored' for r in block)
        hits = sum(r['target_status']=='reached' for r in block)
        hits_times = observed(block,'target_time_ms')
        performance.append(dict(n=n,stratum=stratum,method=method,cases=len({r['case'] for r in block}),
            valid_cases=len(vals),runs=len(block),valid_runs=len(valid),
            mean_cost=mean(vals) if vals else '',sd_between_cases=stdev(vals) if len(vals)>1 else '',
            target_hits=hits,target_censored=censored,target_rate=hits/(hits+censored) if hits+censored else '',
            restricted_mean_target_ms=mean(hits_times) if hits_times else '',
            exact_calls=sum(int(r['exact_calls']) for r in block),
            exact_completed=sum(int(r['exact_completed']) for r in block),
            exact_improvements=sum(int(r['exact_improvements']) for r in block)))
    write(destination/'table7_summary.csv',performance)

    cp_lookup = {(r['case'],r['seed'],r['method'],r['checkpoint_ms']):r for r in cps}
    assert len(cp_lookup) == len(cps)
    contrasts, curves = defaultdict(lambda:defaultdict(list)), defaultdict(lambda:defaultdict(list))
    for row in cps:
        if row['status'] != 'ok':
            continue
        case,seed,method,time = (row[k] for k in ('case','seed','method','checkpoint_ms'))
        n = int(case.split('_')[0][1:])
        initial = float(keyed[case,seed,method]['initial_objective'])
        value = float(row['objective'])
        curves[n,method,int(time)][case].append(100*(1-value/initial))
        reference = cp_lookup[case,seed,'hybrid',time]
        if method != 'hybrid' and reference['status']=='ok':
            baseline = float(reference['objective'])
            for stratum in (row['stratum'],'all'):
                contrasts[n,stratum,method,int(time)][case].append((value-baseline,100*(value-baseline)/value))

    rng = random.Random(608152)
    effects = []
    for (n,stratum,method,time), case_results in sorted(contrasts.items()):
        by_geometry = defaultdict(list)
        for case, values in case_results.items():
            by_geometry[case.split('_')[1]].append(tuple(mean(x) for x in zip(*values)))
        groups = list(by_geometry.values())
        absolute = [[value[0] for value in group] for group in groups]
        percent = [[value[1] for value in group] for group in groups]
        lo,hi = interval(absolute,rng)
        lp,hp = interval(percent,rng)
        avg = mean(value for group in absolute for value in group)
        effects.append(dict(n=n,stratum=stratum,method=method,checkpoint_ms=time,
            paired_cases=len(case_results),paired_runs=sum(map(len,case_results.values())),
            hybrid_saves_cny=avg,ci95_cny_low=lo,ci95_cny_high=hi,
            hybrid_saves_pct=mean(value for group in percent for value in group),ci95_pct_low=lp,ci95_pct_high=hp,
            hybrid_better_cases=sum(value>1e-8 for group in absolute for value in group),
            tied_cases=sum(abs(value)<=1e-8 for group in absolute for value in group),
            hybrid_worse_cases=sum(value<-1e-8 for group in absolute for value in group)))
    write(destination/'paired_effects.csv',effects)

    trajectories = []
    for (n,method,time), case_results in sorted(curves.items()):
        geom = defaultdict(list)
        for case, values in case_results.items():
            geom[case.split('_')[1]].append(mean(values))
        lo,hi=interval(list(geom.values()),rng)
        trajectories.append(dict(n=n,method=method,checkpoint_ms=time,cases=len(case_results),
            runs=sum(map(len,case_results.values())),mean_improvement_pct=mean(mean(vals) for vals in case_results.values()),
            ci95_low=lo,ci95_high=hi))
    write(destination/'figure3_checkpoints.csv',trajectories)
    overall = {'stage':stage,'cases':len(complete),'expected_cases':120,'runs':len(runs),
               'valid':sum(r['status']=='ok' for r in runs),'verified':sum(r['independent_check']=='verified' for r in runs),
               'checkpoints':len(cps),'fixed_final_comparisons':[r for r in effects if r['stratum']=='all' and
                   r['checkpoint_ms']==max(int(x['budget_ms']) for x in runs if x['case'].startswith(f'n{r["n"]}_'))]}
    (destination/'status.json').write_text(json.dumps(overall,indent=2),encoding='utf-8')
    return overall


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('stage',choices=('screen','formal'))
    args=parser.parse_args()
    overall=aggregate(args.stage)
    print(json.dumps(overall))
