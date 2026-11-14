import csv
from collections import defaultdict
from hashlib import sha256
import math
from pathlib import Path
from statistics import mean, stdev
import subprocess
import sys


WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK))
from verify_compare import decode_sorties, independent_evaluation

CASES = WORK / 'formal_cases'
BINARY = WORK / 'compare-build' / 'truck_drone_compare.exe'
JOB = WORK / 's1_alns'
OUTPUT = WORK.parent / 'outputs' / 'experiment-results' / 'model_validation'
SEEDS = (101, 102, 103, 104, 105)
BUDGET_MS = 1000


def read_csv(path):
    with path.open(newline='', encoding='utf-8') as source:
        return list(csv.DictReader(source))


def write_csv(name, rows):
    with (OUTPUT / name).open('w', newline='', encoding='utf-8') as target:
        writer = csv.DictWriter(target, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run(cases):
    all_rows = []
    for n in (16, 25):
        items = [case for case in cases if int(case['n']) == n]
        assert len(items) == 18
        manifest = JOB / f'n{n}.txt'
        result = JOB / f'n{n}'
        lines = [
            'BENCHMARK 1', f'BUDGET_MS {BUDGET_MS}', 'CHECKPOINTS 3 250 500 1000',
            'SEEDS 5 ' + ' '.join(map(str, SEEDS)), 'METHODS 1 hybrid', 'FIXED_Q 3',
            'ORDER_FIRST 6 2', 'VNS 3', 'BOOTSTRAP 100 91251', f'CASES {len(items)}'
        ]
        for case in items:
            path = CASES / case['path']
            lines.append(f'{case["case"]} n{n}_{case["geometry"]} "{path.as_posix()}" -')
        manifest.write_text('\n'.join([*lines, 'END', '']), encoding='ascii')
        if not (result / 'paired.csv').is_file():
            subprocess.run([BINARY, manifest, result], cwd=JOB, check=True, timeout=130)
        observed = read_csv(result / 'runs.csv')
        assert {(r['case'], int(r['seed'])) for r in observed} == {(c['case'], s) for c in items for s in SEEDS}
        assert len(observed) == len(items) * len(SEEDS)
        all_rows.extend(observed)
    return all_rows


def verify_rows(cases, rows):
    fixtures = {c['case']: (CASES / c['path']).read_bytes() for c in cases}
    for row in rows:
        instance = fixtures[row['case']]
        row['instance_sha256'] = sha256(instance).hexdigest()
        row['independently_verified'] = 0
        assert row['status'] in ('ok', 'no_incumbent')
        if row['status'] == 'no_incumbent':
            assert row['objective'] == row['truck_tour'] == ''
            continue
        tour = list(map(int, row['truck_tour'].split()))
        expected = independent_evaluation(instance.decode('ascii'), tour, decode_sorties(row['sorties']))
        assert all(math.isclose(float(row[field]), value, abs_tol=1e-9, rel_tol=1e-9)
                   for field, value in expected.items()), (row['case'], row['seed'])
        row['independently_verified'] = 1
    return rows


def table4(cases, rows):
    runs = defaultdict(list)
    for row in rows:
        runs[row['case']].append(row)
    milp = defaultdict(dict)
    for row in read_csv(OUTPUT / 's1_warm_base_full.csv'):
        milp[row['case']][row['variant']] = row
    assert set(runs) == set(milp) == {case['case'] for case in cases}
    output = []
    for case in cases:
        name, n, geometry = case['case'], int(case['n']), case['geometry']
        successful = [r for r in runs[name] if r['status'] == 'ok']
        costs = [float(r['objective']) for r in successful]
        assert set(milp[name]) == {'base', 'full'}
        base, full = (milp[name][variant] for variant in ('base', 'full'))
        mips = [float(r['objective']) for r in (base, full) if r['objective'] and r['verified'] == '1']
        bounds = [float(r['lower_bound']) for r in (base, full) if r['lower_bound']]
        upper = min(mips) if mips else None
        lower = max(bounds) if bounds else None
        value = min(costs) if costs else None
        full_inc = float(full['objective']) if full['objective'] and full['verified'] == '1' else None
        certified = [float(r['objective']) for r in (base, full) if r['grb_status'] == '2' and r['verified'] == '1']
        if value is not None and lower is not None:
            assert value >= lower - 1e-6
        output.append({
            'case': name, 'n': n, 'geometry': geometry,
            'alns_budget_s': BUDGET_MS / 1000, 'alns_runs': len(runs[name]), 'alns_verified': len(successful),
            'alns_best_cny': value, 'alns_mean_cny': mean(costs) if costs else None,
            'alns_sd_cny': stdev(costs) if len(costs) > 1 else None,
            'alns_mean_elapsed_s': mean(float(r['elapsed_ms']) for r in runs[name]) / 1000,
            'milp_budget_s_per_variant': full['budget_s'],
            'milp_start_cny': full['start_cost'], 'milp_strengthened_incumbent_cny': full_inc,
            'milp_strengthened_lower_bound_cny': full['lower_bound'],
            'milp_strengthened_elapsed_s': full['mip_elapsed_s'], 'milp_strengthened_gap_pct': full['gap_pct'],
            'best_verified_milp_incumbent_cny': upper, 'strongest_milp_lower_bound_cny': lower,
            'certified_optimum_cny': min(certified) if certified else None,
            'alns_best_minus_milp_incumbent_cny': value - upper if value is not None and upper is not None else None,
            'alns_best_gap_upper_bound_pct': 100 * (value - lower) / value
            if value is not None and value > 0 and lower is not None and lower > 0 else None,
        })
    return output


def grouped(cases):
    strata = defaultdict(list)
    for row in cases:
        strata[row['n'], row['geometry']].append(row)
        strata[row['n'], 'all'].append(row)
        strata['all', 'all'].append(row)
    output = []
    for (n, geo), group in strata.items():
        def values(field):
            return [row[field] for row in group if row[field] is not None]

        delta = values('alns_best_minus_milp_incumbent_cny')
        gaps = values('alns_best_gap_upper_bound_pct')
        output.append({
            'n': n, 'geometry': geo, 'cases': len(group),
            'alns_runs': sum(values('alns_runs')), 'alns_verified_runs': sum(values('alns_verified')),
            'alns_mean_best_cny': mean(values('alns_best_cny')),
            'alns_mean_seed_mean_cny': mean(values('alns_mean_cny')),
            'alns_mean_elapsed_s': mean(values('alns_mean_elapsed_s')),
            'verified_milp_incumbents': len(delta), 'certified_optima': len(values('certified_optimum_cny')),
            'alns_better_than_milp_incumbent': sum(x < -1e-7 for x in delta),
            'alns_ties_milp_incumbent': sum(abs(x) <= 1e-7 for x in delta),
            'alns_worse_than_milp_incumbent': sum(x > 1e-7 for x in delta),
            'mean_alns_minus_milp_incumbent_cny': mean(delta) if delta else None,
            'informative_bound_cases': len(gaps),
            'mean_gap_upper_bound_pct': mean(gaps) if gaps else None,
        })
    return output


def main():
    JOB.mkdir(exist_ok=True)
    cases = [case for case in read_csv(CASES / 'instances.csv') if case['group'] == 'S1']
    assert len(cases) == 36
    runs = verify_rows(cases, run(cases))
    per_case = table4(cases, runs)
    write_csv('s1_alns_runs.csv', runs)
    write_csv('s1_alns_table4.csv', per_case)
    write_csv('s1_alns_grouped.csv', grouped(per_case))
    print(f'{len(cases)} cases, {len(runs)} runs, {sum(r["independently_verified"] for r in runs)} verified.')


if __name__ == '__main__':
    main()
