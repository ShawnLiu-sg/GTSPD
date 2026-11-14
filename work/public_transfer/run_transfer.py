from collections import defaultdict
from hashlib import sha1, sha256
from pathlib import Path
from random import Random
import csv
import json
import math
import statistics
import subprocess
from zipfile import ZipFile, ZIP_DEFLATED


HERE = Path(__file__).resolve().parent
RAW = HERE / 'raw'
ROOT = HERE.parent
OUTPUT = ROOT.parent / 'outputs' / 'experiment-results' / 'public_transfer'
BINARY = ROOT / 'compare-build' / 'truck_drone_compare.exe'
INPUTS = HERE / 'cases'
SEEDS = (101, 103, 107)
TRUCK_SPEED = 11.5


def git_sha(blob):
    return sha1(f'blob {len(blob)}\x00'.encode() + blob).hexdigest()


def write_csv(path, rows):
    with path.open('w', newline='', encoding='utf-8') as out:
        writer = csv.DictWriter(out, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path):
    with path.open(newline='', encoding='utf-8') as source:
        return list(csv.DictReader(source))


def input_data(source):
    coords_file = RAW / source['coordinate_path']
    coords_blob = coords_file.read_bytes()
    if git_sha(coords_blob) != source['source_coordinate_blob']:
        trimmed = coords_blob.rstrip(b'\r\n')
        possibilities = [trimmed, trimmed + b'\n', trimmed + b'\r\n']
        coords_blob = next(blob for blob in possibilities if git_sha(blob) == source['source_coordinate_blob'])
    travel_blob = ''.join((RAW / part).read_text(encoding='ascii').rstrip('\r\n')
                          for part in source['travel_parts']).encode('ascii')
    assert git_sha(travel_blob) == source['source_matrix_blob'], source['source_matrix_path']
    coord_text = coords_blob.decode('ascii')
    header, data = coord_text.split('NODE_COORD_SECTION\n', 1)
    count = int(next(line for line in header.splitlines() if line.startswith('DIMENSION')).split(':')[1])
    codes, points = [], []
    for line in data.splitlines():
        if not line.strip() or line.strip() == 'EOF':
            continue
        idx, x, y, code = line.split()
        assert int(idx) == len(codes)
        codes.append(code)
        points.append((float(x), float(y)))
    n = count - 1
    assert n == source['customers'] == len(codes) - 1
    assert len(set(codes)) == len(codes)
    times = json.loads(travel_blob)
    assert set(times) == set(codes)
    for code in codes:
        assert set(times[code]) == set(codes) and times[code][code] == 0
        assert all(math.isfinite(t) and t >= 0 for t in times[code].values())
    route = coords_file.name.replace('_SPP_MD_H.tsp', '')
    return route, n, codes, points, times, coords_blob, travel_blob


def input_files():
    model = (ROOT / 'formal_cases' / 'cases' / 'n100_U_01.txt').read_text(encoding='ascii')
    settings = 'SEARCH' + model.rsplit('SEARCH', 1)[1]
    header = model.split('ORDERS', 1)[0]
    sources = json.loads((RAW / 'sources.json').read_text(encoding='ascii'))
    metadata = []
    for source in sources:
        route, n, codes, points, times, coordinate_blob, travel_blob = input_data(source)
        scenario_seed = int.from_bytes(sha256(route.encode('ascii')).digest()[:8], 'big')
        rng = Random(scenario_seed)
        heavy = set(rng.sample(range(1, n + 1), round(0.20 * n)))
        parcels = [rng.randint(4501, 10000) if i in heavy else rng.randint(1, 4500) for i in range(1, n + 1)]
        truck_capacity = max(600, math.ceil(sum(parcels) / 1000))
        orders = ''.join(f'{i} {grams / 1000:.3f} {int(grams <= 4500)} 1\n'
                         for i, grams in enumerate(parcels, 1))
        road = ''.join(' '.join(f'{times[a][b] * TRUCK_SPEED:.8f}' for b in codes) + '\n' for a in codes)
        air = ''.join(' '.join(f'{math.dist(a, b):.8f}' for b in points) + '\n' for a in points)
        standard = header.replace('CUSTOMERS 100', f'CUSTOMERS {n}').replace('TRUCK 600 ', f'TRUCK {truck_capacity} ')
        standard += 'ORDERS\n' + orders + 'ROAD\n' + road + 'AIR\n' + air + settings
        truck_only = standard.replace('DRONE 3 ', 'DRONE 0 ')
        assert truck_only != standard and sum(parcels) <= truck_capacity * 1000
        for suffix, data in (('joint', standard), ('truck', truck_only)):
            (INPUTS / f'{route}_{suffix}.txt').write_text(data, encoding='ascii', newline='\n')
        budget = 1000 if n <= 130 else 3000
        metadata.append({
            'route_id': route, 'customers': n, 'depot_stop': codes[0], 'budget_ms': budget,
            'scenario_seed': scenario_seed, 'heavy_orders': len(heavy), 'truck_capacity_kg': truck_capacity,
            'total_payload_kg': sum(parcels) / 1000, 'source_coordinates': source['coordinate_path'],
            'source_travel_times': source['source_matrix_path'], 'source_coordinate_blob': git_sha(coordinate_blob),
            'source_matrix_blob': git_sha(travel_blob), 'joint_sha256': sha256(standard.encode()).hexdigest(),
            'truck_only_sha256': sha256(truck_only.encode()).hexdigest()
        })
    return metadata


def run_matrix(metadata):
    runs, checkpoints = [], []
    for row in metadata:
        route, budget = row['route_id'], row['budget_ms']
        for scenario, methods in (('joint', '2 hybrid plain'), ('truck', '1 plain')):
            name = f'{route}_{scenario}'
            output = HERE / 'results' / name
            manifest = HERE / f'{name}.txt'
            stratum = 'up_to_130' if row['customers'] <= 130 else 'over_130'
            manifest.write_text(
                f'BENCHMARK 1\nBUDGET_MS {budget}\nCHECKPOINTS 3 {budget//4} {budget//2} {budget}\n'
                f'SEEDS {len(SEEDS)} {" ".join(map(str, SEEDS))}\nMETHODS {methods}\nFIXED_Q 4\n'
                f'ORDER_FIRST 6 2\nVNS 3\nBOOTSTRAP 100 91251\nCASES 1\n'
                f'{route} {stratum} "cases/{name}.txt" -\nEND\n', encoding='ascii', newline='\n')
            if not (output / 'paired.csv').is_file():
                subprocess.run([BINARY, manifest, output], check=True, cwd=HERE, capture_output=True, timeout=60)
            for filename, records in (('runs.csv', runs), ('checkpoints.csv', checkpoints)):
                for run in read_csv(output / filename):
                    if scenario == 'truck':
                        run['method'] = 'truck_only'
                        if filename == 'runs.csv' and run['status'] == 'ok':
                            assert run['sorties'] == '' and run['drone_energy_kwh'] == '0'
                    records.append(run)
        print(route, row['customers'], budget, flush=True)
    return runs, checkpoints


def summarize(runs):
    per_route = defaultdict(dict)
    for run in runs:
        assert run['status'] in ('ok', 'no_incumbent')
        key = run['case'], run['stratum'], run['seed']
        per_route[key][run['method']] = run
    groups = defaultdict(lambda: defaultdict(list))
    denominators = defaultdict(list)
    for (route, stratum, seed), methods in per_route.items():
        assert set(methods) == {'hybrid', 'plain', 'truck_only'}
        for method in ('plain', 'truck_only'):
            left = methods[method]
            right = methods['hybrid']
            for group in (stratum, 'all'):
                denominators[group, method].append((route, left['status'] == 'ok', right['status'] == 'ok'))
            if left['status'] != 'ok' or right['status'] != 'ok':
                continue
            cost = float(left['objective']) - float(right['objective'])
            pct = 100 * cost / float(left['objective'])
            emission = float(left['emissions_kg_co2e']) - float(right['emissions_kg_co2e'])
            wait = float(left['truck_wait_s']) - float(right['truck_wait_s'])
            for group in (stratum, 'all'):
                groups[group, method][route].append((cost, pct, emission, wait))
    rows = []
    for (stratum, method), counts in sorted(denominators.items()):
        routes = groups[stratum, method]
        assert routes
        means = [tuple(map(statistics.mean, zip(*repeats))) for repeats in routes.values()]
        absolute, relative, emissions, waiting = zip(*means)
        rows.append({
            'stratum': stratum, 'reference_method': method, 'scheduled_routes': len({c[0] for c in counts}),
            'scheduled_pairs': len(counts), 'hybrid_valid_runs': sum(c[2] for c in counts),
            'reference_valid_runs': sum(c[1] for c in counts), 'paired_routes': len(routes),
            'paired_runs': sum(len(r) for r in routes.values()),
            'routes_lower_cost_with_hybrid': sum(x > 1e-8 for x in absolute),
            'mean_hybrid_cny_saving': statistics.mean(absolute),
            'mean_hybrid_pct_saving': statistics.mean(relative),
            'mean_hybrid_emissions_reduction_kg': statistics.mean(emissions),
            'mean_hybrid_wait_reduction_s': statistics.mean(waiting),
        })
    return rows


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    INPUTS.mkdir(parents=True, exist_ok=True)
    metadata = input_files()
    write_csv(OUTPUT / 'routes.csv', metadata)
    runs, checkpoints = run_matrix(metadata)
    assert len(runs) == 13 * len(SEEDS) * 3 and len(checkpoints) == 3 * len(runs)
    write_csv(OUTPUT / 'runs.csv', runs)
    write_csv(OUTPUT / 'checkpoints.csv', checkpoints)
    write_csv(OUTPUT / 'summary.csv', summarize(runs))
    with ZipFile(OUTPUT / 'converted_cases_and_manifests.zip', 'w', compression=ZIP_DEFLATED) as archive:
        for path in (*sorted(INPUTS.glob('*.txt')), *sorted(HERE.glob('RouteID_*.txt'))):
            archive.write(path, path.relative_to(HERE))
    print(f'{len(metadata)} routes, {len(runs)} runs and {len(checkpoints)} checkpoints', flush=True)


if __name__ == '__main__':
    main()
