import csv
from collections import defaultdict
from hashlib import sha256
from itertools import product
from pathlib import Path
import statistics
import subprocess
import time


HERE = Path(__file__).resolve().parent
CASES = HERE.parent / 'formal_cases' / 'cases'
OUTPUT = HERE.parent.parent / 'outputs' / 'experiment-results' / 'mechanism'
SIZES = (50, 100)
GEOMETRIES = ('U', 'R', 'CL')
CASE_COUNT_PER_CELL = 10
SEED = 101
BUDGET_MS = 1500

DESTROY = ('random', 'spatial_relatedness', 'worst_nonlaunch', 'worst_launch', 'edge_based', 'longest_sortie')
REPAIR = ('greedy', 'regret_2', 'random_greedy')
NOISE = ('off', 'on')

RUN_FIELDS = ('case', 'size', 'geometry', 'seed', 'budget_ms', 'instance_sha256',
              'iterations', 'elapsed_s', 'exact_calls', 'exact_completed', 'exact_improvements', 'snapshot')
EVENT_FIELDS = ('case', 'size', 'geometry', 'seed', 'iteration', 'destroy', 'repair', 'noise',
                'requested', 'removed', 'completed', 'accepted', 'improved_current', 'improved_best',
                'destroy_us', 'repair_us')
SNAPSHOT_FIELDS = ('case', 'size', 'geometry', 'seed', 'iteration', 'stop_1', 'stop_2', 'pool_size',
                   'initial_cny', 'within_stop_cny', 'cross_stop_cny', 'enumerated_within_cny',
                   'enumerated_cross_cny', 'initial_wait_s', 'within_stop_wait_s', 'cross_stop_wait_s',
                   'initial_emissions_kg', 'within_stop_emissions_kg', 'cross_stop_emissions_kg',
                   'within_stop_median_us', 'cross_stop_median_us', 'oracle_median_us',
                   'initial_tour', 'initial_sorties', 'within_stop_tour', 'within_stop_sorties',
                   'cross_stop_tour', 'cross_stop_sorties')
OPERATOR_FIELDS = ('size', 'geometry', 'category', 'operator', 'instances', 'selected', 'completed',
                   'accepted', 'improved_current', 'improved_best', 'improved_current_pct_of_completed',
                   'improved_best_pct_of_completed', 'median_stage_us')
REPLAY_FIELDS = ('size', 'geometry', 'scheduled', 'snapshots', 'within_stop_improved', 'cross_stop_improved',
                 'cross_stop_beats_within', 'mean_within_stop_saving_cny', 'mean_cross_stop_saving_cny',
                 'mean_incremental_cross_stop_saving_cny', 'mean_incremental_cross_stop_wait_reduction_s',
                 'mean_incremental_cross_stop_emission_reduction_kg', 'median_cross_stop_us', 'median_oracle_us')


def write_rows(name, fields, rows):
    with (OUTPUT / name).open('w', newline='', encoding='utf-8') as target:
        writer = csv.writer(target)
        writer.writerow(fields)
        writer.writerows(rows)


def run_case(size, geometry, index):
    case = f'n{size}_{geometry}_{index:02}'
    path = CASES / (case + '.txt')
    result = subprocess.run([HERE / 'replay.exe', path, str(SEED), str(BUDGET_MS)],
                            cwd=HERE, check=True, capture_output=True, text=True, timeout=25)
    operators = []
    snapshot = None
    prefix = (case, size, geometry, SEED)
    for line in result.stdout.splitlines():
        kind, *fields = line.split('\t')
        if kind == 'run':
            run = fields
        elif kind == 'operator':
            operators.append((*prefix, *fields))
        elif kind == 'snapshot':
            snapshot = (*prefix, *(field.strip() for field in fields))
        else:
            assert kind == 'no_snapshot', kind
    assert len(run) == 5 and int(run[0]) == sum(int(row[10]) for row in operators)
    if snapshot:
        assert len(snapshot) == len(SNAPSHOT_FIELDS) and snapshot[-2] == snapshot[-4] == snapshot[-6]
    return (*prefix, BUDGET_MS, sha256(path.read_bytes()).hexdigest(), *run,
            'captured' if snapshot else 'none'), operators, snapshot


def operator_groups(events):
    groups = defaultdict(list)
    for event in events:
        case, size, geometry = event[:3]
        for category, offset, choices, duration_offset in (
                ('destroy', 5, DESTROY, 14), ('repair', 6, REPAIR, 15), ('noise', 7, NOISE, 15)):
            label = choices[int(event[offset])]
            row = (case, *(int(x) for x in event[10:14]), float(event[duration_offset]))
            for stratum in ((size, geometry), (size, 'all'), ('all', 'all')):
                groups[(*stratum, category, label)].append(row)
    for key, rows in sorted(groups.items(), key=lambda entry: tuple(map(str, entry[0]))):
        completed = sum(row[1] for row in rows)
        accepted = sum(row[2] for row in rows)
        current = sum(row[3] for row in rows)
        best = sum(row[4] for row in rows)
        yield (*key, len({row[0] for row in rows}), len(rows), completed, accepted, current, best,
               100 * current / completed if completed else None, 100 * best / completed if completed else None,
               statistics.median(row[5] for row in rows if row[1]) if completed else None)


def replay_groups(runs, snapshots):
    counts = defaultdict(int)
    for _, size, geometry, *_ in runs:
        for stratum in ((size, geometry), (size, 'all'), ('all', 'all')):
            counts[stratum] += 1
    groups = defaultdict(list)
    for row in snapshots:
        size, geometry = row[1:3]
        for stratum in ((size, geometry), (size, 'all'), ('all', 'all')):
            groups[stratum].append(row)
    epsilon = 1e-8
    for stratum, scheduled in sorted(counts.items(), key=lambda entry: tuple(map(str, entry[0]))):
        rows = groups[stratum]
        if not rows:
            yield (*stratum, scheduled, 0, 0, 0, 0, None, None, None, None, None, None, None)
            continue
        within = [float(row[8]) - float(row[9]) for row in rows]
        cross = [float(row[8]) - float(row[10]) for row in rows]
        extra = [float(row[9]) - float(row[10]) for row in rows]
        wait = [float(row[14]) - float(row[15]) for row in rows]
        emissions = [float(row[17]) - float(row[18]) for row in rows]
        yield (*stratum, scheduled, len(rows), sum(x > epsilon for x in within), sum(x > epsilon for x in cross),
               sum(x > epsilon for x in extra), statistics.mean(within), statistics.mean(cross), statistics.mean(extra),
               statistics.mean(wait), statistics.mean(emissions), statistics.median(float(row[20]) for row in rows),
               statistics.median(float(row[21]) for row in rows))


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    runs, events, snapshots = [], [], []
    started = time.perf_counter()
    for size, geometry, index in product(SIZES, GEOMETRIES, range(1, CASE_COUNT_PER_CELL + 1)):
        run, operator_events, snapshot = run_case(size, geometry, index)
        runs.append(run)
        events.extend(operator_events)
        if snapshot:
            snapshots.append(snapshot)
        print(f'{run[0]}  iterations={run[6]}  snapshot={run[-1]}', flush=True)
    write_rows('instrumented_runs.csv', RUN_FIELDS, runs)
    write_rows('operator_events.csv', EVENT_FIELDS, events)
    write_rows('operator_summary.csv', OPERATOR_FIELDS, operator_groups(events))
    write_rows('neighborhood_replay.csv', SNAPSHOT_FIELDS, snapshots)
    write_rows('neighborhood_summary.csv', REPLAY_FIELDS, replay_groups(runs, snapshots))
    print(f'{len(runs)} cases, {len(events)} operator events, {len(snapshots)} replayed neighborhoods, '
          f'{time.perf_counter() - started:.1f} s total.', flush=True)


if __name__ == '__main__':
    main()
