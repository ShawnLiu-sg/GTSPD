from dataclasses import dataclass
from pathlib import Path
import csv
import json
import math
import random


root = Path(__file__).resolve().parent / 'formal_cases'
params = {
    16: (8, 2.8, .42, 3, .72, 4.8, 3.2),
    25: (12, 4.2, .63, 4, 1.08, 7.2, 3.6),
    50: (18, 6.3, .945, 8, 1.62, 10.8, 3.6),
    100: (20, 10.5, 1.575, 10, 1.60, 12, 4),
}
speed = 11.5
fuel_scale = .0234 / (737 * 1000 * .45 * .45)
fuel_empty = .25 * 38.34 * 4.5 * .0234 / 737 + fuel_scale * speed * (
    .5 * .6 * 6 * 1.204 * speed**2 + 3500 * 9.81 * .01)
fuel_payload = fuel_scale * speed * 9.81 * .01


def positions(n, geometry, rng):
    side, radius, sigma_ring, clusters, sigma_cluster, grid, separation = params[n]
    if geometry == 'U':
        return [(rng.uniform(-side / 2, side / 2), rng.uniform(-side / 2, side / 2)) for _ in range(n)]
    if geometry == 'R':
        points = []
        for _ in range(n):
            distance = rng.gauss(radius, sigma_ring)
            while distance < 0:
                distance = rng.gauss(radius, sigma_ring)
            angle = rng.uniform(0, 2 * math.pi)
            points.append((distance * math.cos(angle), distance * math.sin(angle)))
        return points
    count = int(grid / separation) + 1
    axis = [-grid / 2 + index * grid / (count - 1) for index in range(count)]
    centers = rng.sample([(x, y) for x in axis for y in axis], clusters)
    points = []
    for index in range(n):
        center_x, center_y = centers[index % clusters]
        radius = abs(rng.gauss(0, sigma_cluster))
        angle = rng.uniform(0, 2 * math.pi)
        points.append((center_x + radius * math.cos(angle), center_y + radius * math.sin(angle)))
    rng.shuffle(points)
    return points


@dataclass(frozen=True)
class Case:
    group: str
    n: int
    geometry: str
    replica: int
    seed: int

    @property
    def key(self):
        prefix = 'train_' if self.group == 'tuning' else ''
        return f'{prefix}n{self.n}_{self.geometry}_{self.replica:02}'

    def write(self):
        rng = random.Random(self.seed)
        points = positions(self.n, self.geometry, rng)
        heavy = set(rng.sample(range(1, self.n + 1), round(.20 * self.n)))
        lines = ['STDRP 1', f'CUSTOMERS {self.n}',
                 f'TRUCK 600 {speed} {fuel_empty:.14g} {fuel_payload:.14g}',
                 'DRONE 3 4.5 0.510 11.5 1179.31 184.54',
                 'COSTS 0.005693 0.000684 0.264 0.005 3.24 0.577', 'ORDERS']
        for i in range(1, self.n + 1):
            mass = rng.randint(4501, 10000) if i in heavy else rng.randint(1, 4500)
            # Keep payload eligibility dynamic under the scenario's capacity limit.
            lines.append(f'{i} {mass/1000:.3f} 1 1')
        lines.extend(['COORDINATES', '0 0 0'])
        for i, (x, y) in enumerate(points, 1):
            lines.append(f'{i} {1000*x:.6f} {1000*y:.6f}')
        lines.extend(['SEARCH 101 1000000 15000', f'DESTROY 2 {round(.16*self.n)} 0.13 26',
                      'ANNEAL 4 0.9995 0.0000001', 'ADAPT 20 0.3 0.08 8 4 2 1',
                      'EXACT 4 8 45', 'END'])
        path = root / 'cases' / f'{self.key}.txt'
        path.write_text('\n'.join(lines) + '\n', encoding='ascii')
        return dict(case=self.key,group=self.group,n=self.n,geometry=self.geometry,replica=self.replica,
                    instance_seed=self.seed,path=f'cases/{path.name}')


if __name__ == '__main__':
    (root / 'cases').mkdir(parents=True, exist_ok=True)
    records = []
    for group, sizes, replicas, base in [('S1', (16,25), 6, 520000),
                                         ('tuning', (50,100), 5, 480000),
                                         ('main', (50,100), 20, 710000)]:
        for n in sizes:
            for index, geometry in enumerate(('U', 'R', 'CL')):
                for replica in range(1, replicas + 1):
                    records.append(Case(group,n,geometry,replica,base + n*100 + index*30 + replica).write())
    with (root / 'instances.csv').open('w', newline='', encoding='ascii') as out:
        writer = csv.DictWriter(out, list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    (root / 'parameters.json').write_text(json.dumps({
        'truck_speed_m_s': speed, 'truck_capacity_kg': 600,
        'truck_empty_l_s': fuel_empty, 'truck_increment_l_s_kg': fuel_payload,
        'drone_speed_m_s': 11.5, 'drone_capacity_kg': 4.5, 'usable_battery_kwh': .510,
        'drone_empty_W': 1179.31, 'drone_increment_W_kg': 184.54, 'drones': 3,
        'truck_cost_cny_m': .005693, 'drone_cost_cny_m': .000684, 'carbon_cny_kg': .264,
        'wait_cny_s': .005, 'fuel_kgCO2e_l': 3.24, 'electricity_kgCO2e_battery_kwh': .577,
        'geometry': params, 'coordinate_units': 'm', 'synthetic_launch_allowed_all_stops': True,
        'heavy_orders': 'round(0.2*n), uniform 0.001 kg grid on [4.501,10.000] kg',
        'light_orders': 'uniform 0.001 kg grid on [0.001,4.500] kg',
        'notes': 'Truck rates from manuscript A.6/B.1. Emission factors and costs are declared reference scenario inputs.'
    }, indent=2), encoding='utf-8')
    print(f'{len(records)} instances created at {root}')
