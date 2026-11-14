import argparse
import csv
from hashlib import sha256
import json
import math
from pathlib import Path
import statistics
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parent
METHODS = ('hybrid', 'plain', 'fixed_q', 'nonadaptive', 'within_stop', 'order_first', 'vns')
SEEDS = (42, 43)
CHECKPOINTS = (100, 250, 500)
BUDGET = 500
CASES = {'reachable': 1000000.0, 'censored': 0.0}


def equal(got, expected):
    assert math.isclose(float(got), expected, rel_tol=1e-9, abs_tol=1e-9), (got, expected)


def manifest(checkpoints=CHECKPOINTS, methods=METHODS):
    return f"""BENCHMARK 1
BUDGET_MS {BUDGET}
CHECKPOINTS {len(checkpoints)} {' '.join(map(str, checkpoints))}
SEEDS {len(SEEDS)} {' '.join(map(str, SEEDS))}
METHODS {len(methods)} {' '.join(methods)}
FIXED_Q 4
ORDER_FIRST 5 2
VNS 2
BOOTSTRAP 1000 2026
CASES 2
reachable demo "case inputs/with drone.txt" 1000000
censored demo "case inputs/truck only.txt" 0
END
"""


def csv_rows(directory, name, fields):
    with (directory / (name + '.csv')).open(newline='', encoding='utf-8-sig') as source:
        reader = csv.DictReader(source)
        assert reader.fieldnames == fields.split(','), (name, reader.fieldnames)
        rows = list(reader)
    assert all(None not in row and None not in row.values() for row in rows), name
    return rows


def unique_index(rows, key):
    indexed = {key(row): row for row in rows}
    assert len(indexed) == len(rows), 'Duplicate rows'
    return indexed


def run_key(row):
    return row['case'], int(row['seed']), row['method']


def independent_evaluation(instance, tour, sorties):
    lines = instance.splitlines()
    values = lambda label: list(map(float, next(s for s in lines if s.startswith(label + ' ')).split()[1:]))
    truck_cap, truck_speed, truck_empty, truck_mass_rate = values('TRUCK')
    drones, drone_cap, battery, drone_speed, empty_w, mass_w = values('DRONE')
    road_price, flight_price, carbon, wait_price, fuel_factor, power_factor = values('COSTS')
    orders = {}
    for line in lines[lines.index('ORDERS') + 1:lines.index('COORDINATES')]:
        node, weight, eligible, launch = line.split()
        orders[int(node)] = (float(weight), int(eligible), int(launch))
    coords = {}
    for line in lines[lines.index('COORDINATES') + 1:]:
        if line.startswith('SEARCH'):
            break
        node, x, y = line.split()
        coords[int(node)] = (float(x), float(y))

    assert tour[0] == tour[-1] == 0 and len(set(tour[1:-1])) == len(tour) - 2
    assert all(node in orders for node in tour[1:-1])
    coverage = list(tour[1:-1])
    launches = {}
    truck_dist = flight_dist = truck_fuel = drone_energy = 0.0
    for anchor, customers in sorties:
        assert anchor in tour[1:-1] and orders[anchor][2] and customers
        assert all(node in orders and orders[node][1] for node in customers)
        coverage.extend(customers)
        payload = sum(orders[node][0] for node in customers)
        assert payload <= drone_cap + 1e-9
        remaining = payload
        path = [anchor, *customers, anchor]
        distance = energy = 0.0
        for first, second in zip(path, path[1:]):
            metres = math.dist(coords[first], coords[second])
            distance += metres
            energy += (empty_w + remaining * mass_w) * metres / drone_speed / 3600000
            if second != anchor:
                remaining -= orders[second][0]
        assert energy <= battery + 1e-9
        flight_dist += distance
        drone_energy += energy
        launches.setdefault(anchor, []).append((payload, distance / drone_speed))
    assert sorted(coverage) == sorted(orders), (tour, sorties)
    assert all(len(scheduled) <= drones for scheduled in launches.values())
    wait = sum(max(duration for _, duration in scheduled) for scheduled in launches.values())
    remaining = sum(order[0] for order in orders.values())
    assert remaining <= truck_cap
    for first, second in zip(tour, tour[1:]):
        metres = sum(abs(a - b) for a, b in zip(coords[first], coords[second]))
        truck_dist += metres
        truck_fuel += (truck_empty + remaining * truck_mass_rate) * metres / truck_speed
        if second:
            remaining -= orders[second][0] + sum(mass for mass, _ in launches.get(second, []))
    emissions = fuel_factor * truck_fuel + power_factor * drone_energy
    costs = {
        'operating_cost': road_price * truck_dist + flight_price * flight_dist,
        'carbon_cost': carbon * emissions,
        'waiting_cost': wait_price * wait,
    }
    return {
        **costs,
        'objective': sum(costs.values()),
        'emissions_kg_co2e': emissions,
        'truck_distance_m': truck_dist,
        'truck_fuel_l': truck_fuel,
        'drone_distance_m': flight_dist,
        'drone_energy_kwh': drone_energy,
        'truck_wait_s': wait,
    }


def decode_sorties(text):
    sorties = []
    for item in text.split('|') if text else ():
        anchor, nodes = item.split(':')
        anchor = int(anchor)
        sorties.append((anchor, list(map(int, nodes.split()))))
    return sorties


RUN_FIELDS = ('case,stratum,seed,method,budget_ms,status,objective,initial_objective,elapsed_ms,iterations,'
              'exact_calls,exact_completed,exact_improvements,operating_cost,carbon_cost,waiting_cost,'
              'emissions_kg_co2e,truck_distance_m,truck_fuel_l,drone_distance_m,drone_energy_kwh,truck_wait_s,'
              'target,target_status,target_time_ms,truck_tour,sorties')
CHECKPOINT_FIELDS = 'case,stratum,seed,method,checkpoint_ms,status,objective'
SUMMARY_FIELDS = ('stratum,method,checkpoint_ms,instances,valid_instances,runs,valid_runs,mean_objective,'
                  'sd_instance_objective,target_runs,target_hits,target_censored')
PAIRED_FIELDS = 'stratum,method,checkpoint_ms,paired_instances,paired_runs,mean_delta,ci_low,ci_high'


def check_outputs(output, instances):
    runs = unique_index(csv_rows(output, 'runs', RUN_FIELDS), run_key)
    checkpoints = unique_index(csv_rows(output, 'checkpoints', CHECKPOINT_FIELDS),
                               lambda row: (*run_key(row), int(row['checkpoint_ms'])))
    summary = unique_index(csv_rows(output, 'summary', SUMMARY_FIELDS),
                           lambda row: (row['method'], int(row['checkpoint_ms'])))
    paired = unique_index(csv_rows(output, 'paired', PAIRED_FIELDS),
                          lambda row: (row['method'], int(row['checkpoint_ms'])))
    assert set(runs) == {(case, seed, method) for case in CASES for seed in SEEDS for method in METHODS}
    assert set(checkpoints) == {(*key, time) for key in runs for time in CHECKPOINTS}
    assert set(summary) == {(method, time) for method in METHODS for time in CHECKPOINTS}
    assert set(paired) == {(method, time) for method in METHODS[1:] for time in CHECKPOINTS}
    assert all(row['stratum'] == 'demo' for group in (runs, checkpoints, summary, paired) for row in group.values())

    has_drone_sortie = False
    for (case, seed, method), row in runs.items():
        assert row['status'] == 'ok' and int(row['budget_ms']) == BUDGET
        assert float(row['objective']) <= float(row['initial_objective']) + 1e-9
        assert float(row['elapsed_ms']) >= 0 and int(row['iterations']) >= 0
        assert 0 <= int(row['exact_improvements']) <= int(row['exact_calls'])
        assert 0 <= int(row['exact_completed']) <= int(row['exact_calls'])
        if method == 'plain':
            assert int(row['exact_calls']) == int(row['exact_completed']) == int(row['exact_improvements']) == 0

        sorties = decode_sorties(row['sorties'])
        has_drone_sortie |= bool(sorties)
        physical = independent_evaluation(instances[case], list(map(int, row['truck_tour'].split())), sorties)
        for field, value in physical.items():
            equal(row[field], value)

        history = [checkpoints[case, seed, method, time] for time in CHECKPOINTS]
        assert all(snapshot['status'] == 'ok' for snapshot in history)
        objectives = [float(snapshot['objective']) for snapshot in history]
        assert all(first >= second - 1e-9 for first, second in zip(objectives, objectives[1:]))
        equal(objectives[-1], physical['objective'])
        target = CASES[case]
        equal(row['target'], target)
        target_time = float(row['target_time_ms'])
        if case == 'reachable':
            assert row['target_status'] == 'reached' and 0 <= target_time <= float(row['elapsed_ms']) + 1e-6
            assert all(value <= target for value, time in zip(objectives, CHECKPOINTS) if target_time <= time)
        else:
            assert row['target_status'] == 'right_censored' and all(value > target for value in objectives)
            equal(target_time, BUDGET)
    assert has_drone_sortie

    for (method, time), row in summary.items():
        case_means = [statistics.mean(float(checkpoints[case, seed, method, time]['objective'])
                                      for seed in SEEDS) for case in CASES]
        assert tuple(int(row[k]) for k in ('instances', 'valid_instances', 'runs', 'valid_runs')) == (2, 2, 4, 4)
        equal(row['mean_objective'], statistics.mean(case_means))
        equal(row['sd_instance_objective'], statistics.stdev(case_means))
        if time == BUDGET:
            assert tuple(int(row[k]) for k in ('target_runs', 'target_hits', 'target_censored')) == (4, 2, 2)
        else:
            assert all(row[k] in ('', '0') for k in ('target_runs', 'target_hits', 'target_censored'))

    for (method, time), row in paired.items():
        differences = [statistics.mean(float(checkpoints[case, seed, method, time]['objective']) -
                                       float(checkpoints[case, seed, 'hybrid', time]['objective'])
                                       for seed in SEEDS) for case in CASES]
        assert (int(row['paired_instances']), int(row['paired_runs'])) == (2, 4)
        equal(row['mean_delta'], statistics.mean(differences))
        equal(row['ci_low'], min(differences))
        equal(row['ci_high'], max(differences))
    return len(runs), len(checkpoints), len(summary), len(paired)


def invoke(binary, scratch, name, contents, accepted=True):
    path = scratch / (name + '.txt')
    path.write_text(contents, encoding='ascii')
    output = scratch / name
    result = subprocess.run([binary, path, output], text=True, capture_output=True, cwd=ROOT, timeout=60)
    if accepted:
        assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
    else:
        assert result.returncode != 0 and (result.stdout or result.stderr), (name, result)
        assert not (output / 'runs.csv').exists(), name
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('binary', type=lambda path: Path(path).resolve())
    parser.add_argument('--legacy', type=lambda path: Path(path).resolve())
    args = parser.parse_args()
    original = (ROOT / 'compare' / 'example.txt').read_text(encoding='ascii')
    instances = {
        'reachable': original,
        'censored': original.replace('DRONE 2 ', 'DRONE 0 '),
    }
    original_hash = sha256(args.legacy.read_bytes()).digest() if args.legacy else None
    with tempfile.TemporaryDirectory(prefix='compare-qa-', dir=ROOT) as temporary:
        scratch = Path(temporary)
        paths = scratch / 'case inputs'
        paths.mkdir()
        (paths / 'with drone.txt').write_text(instances['reachable'], encoding='ascii')
        (paths / 'truck only.txt').write_text(instances['censored'], encoding='ascii')
        output = invoke(args.binary, scratch, 'complete', manifest())
        counts = check_outputs(output, instances)

        only_plain = manifest(checkpoints=(250,), methods=('plain',))
        disabled = only_plain.replace('1000000', '-')
        output = invoke(args.binary, scratch, 'disabled', disabled)
        disabled_rows = csv_rows(output, 'runs', RUN_FIELDS)
        assert all(row['target_status'] == 'disabled' and row['target'] == row['target_time_ms'] == ''
                   for row in disabled_rows if row['case'] == 'reachable')
        assert {int(row['checkpoint_ms']) for row in csv_rows(output, 'checkpoints', CHECKPOINT_FIELDS)} == {250, BUDGET}
        invoke(args.binary, scratch, 'duplicate-method', manifest(methods=('hybrid', 'hybrid')), accepted=False)
        invoke(args.binary, scratch, 'duplicate-case', manifest().replace('censored demo', 'reachable demo'), accepted=False)
        invoke(args.binary, scratch, 'duplicate-seed', manifest().replace('SEEDS 2 42 43', 'SEEDS 2 42 42'), accepted=False)
        invoke(args.binary, scratch, 'decreasing-checkpoints', manifest(checkpoints=(250, 100, 500)), accepted=False)
        invoke(args.binary, scratch, 'missing-instance', manifest().replace('truck only.txt', 'missing.txt'), accepted=False)
        if args.legacy:
            legacy = subprocess.run([args.legacy, paths / 'with drone.txt', '--plain'], text=True,
                                    capture_output=True, cwd=ROOT, check=True, timeout=10)
            solution = json.loads(legacy.stdout)
            sorties = [(item['anchor'], item['route'][1:-1]) for item in solution['sorties']]
            physical = independent_evaluation(original, solution['truck_tour'], sorties)
            for field, value in physical.items():
                equal(solution[field], value)
            assert sha256(args.legacy.read_bytes()).digest() == original_hash
    print(f'Validated {counts[0]} runs, {counts[1]} checkpoints, {counts[2]} summaries and {counts[3]} paired rows.')
    print('Independent route evaluation, censoring, checkpoint append, and five malformed-input cases passed.')


if __name__ == '__main__':
    main()
