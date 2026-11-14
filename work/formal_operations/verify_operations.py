import argparse
import csv
import json
import math
import pathlib
from collections import Counter

from run_operations import CASES, OUTPUT, make_scenarios, selected_cases


def read_case(path):
    lines = path.read_text().splitlines()
    n = int(lines[1].split()[1])
    truck = list(map(float, lines[2].split()[1:]))
    drone = list(map(float, lines[3].split()[1:]))
    costs = list(map(float, lines[4].split()[1:]))
    orders = {}
    for row in lines[6:6 + n]:
        node, mass, eligible, launch = row.split()
        orders[int(node)] = (float(mass), bool(int(eligible)), bool(int(launch)))
    assert lines[6 + n] == "COORDINATES"
    xy = {}
    for row in lines[7 + n:8 + 2 * n]:
        node, x, y = row.split()
        xy[int(node)] = (float(x), float(y))
    return truck, drone, costs, orders, xy


def physical_audit(row, data):
    truck, drone, costs, orders, xy = data
    truck_capacity, truck_speed, empty_fuel, load_fuel = truck
    _, _, _, _, p0, p1 = drone
    road_rate, air_rate, _, _, diesel_factor, _ = costs
    if row["speed"] == 13.41:
        p0, p1 = 1238.9252876087053, 185.4903714817606

    def distance(i, j, air):
        x1, y1 = xy[i]
        x2, y2 = xy[j]
        if air:
            return math.hypot(x2 - x1, y2 - y1)
        return abs(x2 - x1) + abs(y2 - y1)

    if not row["planning"]:
        assert not row["plan"] and not row["physical"]
        return

    def evaluate(p0, p1, road_rate, air_rate, wait_price):
        plan = row["plan"]
        observed = Counter()
        truck_load = sum(mass for mass, _, _ in orders.values())
        assert truck_load <= truck_capacity
        last = 0
        truck_m = fuel = air_m = energy = wait_s = 0.0
        overruns = 0
        maximum_overrun = 0.0
        for stop, *flights in [*plan, [0]]:
            road_m = distance(last, stop, False)
            truck_m += road_m
            fuel += (empty_fuel + load_fuel * truck_load) * road_m / truck_speed
            if stop != 0:
                observed[stop] += 1
                truck_load -= orders[stop][0]
            assert len(flights) <= row["drones"]
            if flights:
                assert orders[stop][2]
            duration = 0.0
            for flight in flights:
                assert len(flight) >= 1
                if row["policy"] == "single_drop":
                    assert len(flight) == 1
                payload = sum(orders[node][0] for node in flight)
                assert payload <= row["capacity"] + 1e-10
                truck_load -= payload
                previous = stop
                sortie_energy = sortie_m = 0.0
                for node in [*flight, stop]:
                    leg_m = distance(previous, node, True)
                    sortie_m += leg_m
                    sortie_energy += (p0 + p1 * payload) * leg_m / (row["speed"] * 3600000)
                    if node != stop:
                        assert orders[node][1]
                        observed[node] += 1
                        payload -= orders[node][0]
                    previous = node
                overrun = sortie_energy - row["battery"]
                if overrun > 1e-10:
                    overruns += 1
                    maximum_overrun = max(maximum_overrun, overrun)
                air_m += sortie_m
                energy += sortie_energy
                duration = max(duration, sortie_m / row["speed"])
            wait_s += duration
            last = stop
        assert observed == Counter({node: 1 for node in orders}), row["case"]
        assert abs(truck_load) < 1e-9
        emissions = fuel * diesel_factor + energy * row["grid"]
        operating = road_rate * truck_m + air_rate * air_m
        carbon = row["carbon"] * emissions
        waiting = wait_price * wait_s
        return {"objective": operating + carbon + waiting, "operating_cost": operating,
                "carbon_cost": carbon, "waiting_cost": waiting, "emissions_kg_co2e": emissions,
                "truck_m": truck_m, "air_m": air_m, "fuel_l": fuel, "air_kwh": energy,
                "wait_s": wait_s}, overruns, maximum_overrun

    def equal(expected, actual):
        assert expected.keys() == actual.keys()
        for key in expected:
            assert math.isclose(expected[key], actual[key], rel_tol=1e-10, abs_tol=1e-10), (
                row["case"], row["name"], key, expected[key], actual[key])

    def physical_error(row, overs):
        return row["case"], row["name"], row["battery_overrun_count"], overs

    full, overs, maximum = evaluate(p0, p1, road_rate, air_rate, row["wait"])
    assert overs == row["battery_overrun_count"], physical_error(row, overs)
    assert math.isclose(maximum, row["max_battery_overrun_kwh"], rel_tol=1e-10, abs_tol=1e-10)
    if overs:
        assert not row["physical"]
    else:
        equal(full, row["physical"])
    planning_rate = 0 if row["policy"] == "emissions_only" else 1
    planning_wait = 0 if row["policy"] in ("emissions_only", "no_wait") else row["wait"]
    if row["policy"] == "constant_energy":
        p0, p1 = p0 + 2.25 * p1, 0
    planning, planning_overs, _ = evaluate(p0, p1, road_rate * planning_rate,
                                         air_rate * planning_rate, planning_wait)
    assert planning_overs == 0
    equal(planning, row["planning"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--budget-ms", type=int, default=350)
    parser.add_argument("--seeds", type=int, nargs="+", default=[42])
    parser.add_argument("--output", type=pathlib.Path, default=OUTPUT)
    parser.add_argument("--cases", type=pathlib.Path, default=CASES)
    parser.add_argument("--confirmation", action="store_true")
    parser.add_argument("--require-all", action="store_true")
    args = parser.parse_args()
    seed_label = "_".join(map(str, args.seeds))
    path = args.output / f"operating_routes_ms{args.budget_ms}_seed{seed_label}.jsonl"
    data = {p.stem: read_case(args.cases / p.name) for p in selected_cases()}
    scenarios = [s for s in make_scenarios() if not args.confirmation or s.family == "policy"
                 or s.name in ("reference", "fleet_0")]
    expected = {(s.name, case, seed) for s in scenarios for case in data for seed in args.seeds}
    keys = set()
    counter = Counter()
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            key = row["name"], row["case"], row["seed"]
            assert key in expected and key not in keys
            keys.add(key)
            assert row["budget_ms"] == args.budget_ms and row["seed"] in args.seeds
            physical_audit(row, data[row["case"]])
            counter["audited"] += 1
            counter["planning_valid"] += bool(row["planning"])
            counter["physical_valid"] += bool(row["physical"])
            counter["cases_with_overrun"] += row["battery_overrun_count"] > 0
    if args.require_all:
        assert keys == expected
        with (args.output / "runs.csv").open(encoding="utf-8", newline="") as stream:
            assert {(row["name"], row["case"], int(row["seed"])) for row in csv.DictReader(stream)} == keys
    print(json.dumps({**counter, "expected": len(expected)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
