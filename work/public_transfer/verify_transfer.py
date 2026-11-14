import csv
from collections import Counter
from hashlib import sha256
from itertools import pairwise
from pathlib import Path
from zipfile import ZipFile
import math


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'outputs' / 'experiment-results' / 'public_transfer'
CASE_DIR = ROOT / 'work' / 'public_transfer' / 'cases'


def rows(name):
    with (OUT / f'{name}.csv').open(newline='', encoding='utf-8') as source:
        return list(csv.DictReader(source))


def near(a, b):
    return math.isclose(float(a), b, abs_tol=1e-8, rel_tol=1e-10)


def physical(input_file, row):
    content = input_file.read_text(encoding='ascii')
    header, remaining = content.split('ORDERS\n', 1)
    config = {line.split()[0]: tuple(map(float, line.split()[1:])) for line in header.splitlines()}
    masses, permitted = {}, {}
    orders, remaining = remaining.split('ROAD\n', 1)
    for line in orders.splitlines():
        node, mass, can_fly, launch = line.split()
        masses[int(node)] = float(mass)
        permitted[int(node)] = int(can_fly), int(launch)
    roads, remaining = remaining.split('AIR\n', 1)
    flights, _ = remaining.split('SEARCH', 1)
    road = [list(map(float, line.split())) for line in roads.splitlines()]
    air = [list(map(float, line.split())) for line in flights.splitlines()]
    cap, truck_v, fuel0, fuelm = config['TRUCK']
    drone_count, drone_cap, battery, air_v, power0, powerm = config['DRONE']
    road_cost, air_cost, carbon_price, waiting_price, fuel_co2, energy_co2 = config['COSTS']
    tour = tuple(map(int, row['truck_tour'].split()))
    assert tour[0] == tour[-1] == 0
    seen = list(tour[1:-1])
    sorties = {}
    wait = energy = metres_air = 0.0
    for flight in row['sorties'].split('|') if row['sorties'] else ():
        anchor, visits = flight.split(':')
        anchor = int(anchor)
        visits = tuple(map(int, visits.split()))
        assert anchor in tour[1:-1] and permitted[anchor][1] and visits
        assert all(permitted[i][0] for i in visits)
        seen.extend(visits)
        mass = sum(masses[i] for i in visits)
        assert mass <= drone_cap + 1e-9
        rem = mass
        used = metres = 0.0
        for i, j in pairwise((anchor, *visits, anchor)):
            length = air[i][j]
            metres += length
            used += (power0 + powerm * rem) * length / air_v / 3600000.0
            if j != anchor:
                rem -= masses[j]
        assert used <= battery + 1e-9
        energy += used
        metres_air += metres
        sorties.setdefault(anchor, []).append((mass, metres / air_v))
    assert sorted(seen) == sorted(masses)
    assert all(len(items) <= drone_count for items in sorties.values())
    rem = sum(masses.values())
    assert rem <= cap + 1e-9
    fuel = metres_road = 0.0
    for i, j in pairwise(tour):
        metres_road += road[i][j]
        fuel += (fuel0 + fuelm * rem) * road[i][j] / truck_v
        if j:
            flights_at_stop = sorties.get(j, ())
            wait += max((duration for _, duration in flights_at_stop), default=0)
            rem -= masses[j] + sum(mass for mass, _ in flights_at_stop)
    assert near(rem, 0)
    emissions = fuel_co2 * fuel + energy_co2 * energy
    operating = road_cost * metres_road + air_cost * metres_air
    expected = {
        'truck_distance_m': metres_road, 'drone_distance_m': metres_air, 'truck_fuel_l': fuel,
        'drone_energy_kwh': energy, 'truck_wait_s': wait, 'emissions_kg_co2e': emissions,
        'operating_cost': operating, 'carbon_cost': carbon_price * emissions,
        'waiting_cost': waiting_price * wait, 'objective': operating + carbon_price * emissions + waiting_price * wait
    }
    assert all(near(row[field], val) for field, val in expected.items()), (row['case'], row['method'], row['seed'])


def main():
    sources = rows('routes')
    runs = rows('runs')
    checkpoints = rows('checkpoints')
    by_key = {(r['case'], r['seed'], r['method']): r for r in runs}
    assert len(sources) == 13 and len(by_key) == len(runs) == 117 and len(checkpoints) == 351
    statuses = Counter(row['status'] for row in runs)
    assert statuses == {'ok': 108, 'no_incumbent': 9}
    for source in sources:
        route = source['route_id']
        for suffix, sha_field in (('joint', 'joint_sha256'), ('truck', 'truck_only_sha256')):
            path = CASE_DIR / f'{route}_{suffix}.txt'
            assert sha256(path.read_bytes()).hexdigest() == source[sha_field]
        joint = (CASE_DIR / f'{route}_joint.txt').read_text(encoding='ascii')
        truck = (CASE_DIR / f'{route}_truck.txt').read_text(encoding='ascii')
        assert joint.replace('DRONE 3 ', 'DRONE 0 ') == truck
    for run in runs:
        scenario = 'truck' if run['method'] == 'truck_only' else 'joint'
        if run['status'] == 'ok':
            physical(CASE_DIR / f'{run["case"]}_{scenario}.txt', run)
        else:
            assert not run['objective'] and not run['truck_tour']
    for checkpoint in checkpoints:
        final = by_key[checkpoint['case'], checkpoint['seed'], checkpoint['method']]
        if checkpoint['checkpoint_ms'] == final['budget_ms']:
            assert checkpoint['status'] == final['status'] and checkpoint['objective'] == final['objective']
    with ZipFile(OUT / 'converted_cases_and_manifests.zip') as archive:
        assert archive.testzip() is None and len(archive.namelist()) == 52
    print('Verified 13 matched scenarios, 108 independently costed plans, 9 censored calls, and 52 archived inputs.')


if __name__ == '__main__':
    main()
