import argparse
import itertools
import json
import pathlib
from dataclasses import dataclass

from run_operations import CASES, OUTPUT, write_csv
from verify_operations import read_case


DRONE_MASS = 9.5
FEE_GRID = (0, 2, 5, 10, 20, 40, 80, 120, 160)


@dataclass(frozen=True)
class FleetPlan:
    drones: int
    source: str
    peak_drones_used: int
    hardware_mass: float
    loaded_mass: float
    fuel_increment: float
    carbon_cost_increment: float
    operating_objective: float

    def total(self, fee):
        return self.operating_objective + self.drones * fee


def deployment_plans(rows, case_path):
    truck, _, costs, orders, _ = read_case(case_path)
    truck_capacity, truck_speed, _, fuel_load_rate = truck
    _, _, carbon_rate, _, fuel_factor, _ = costs
    parcels = sum(mass for mass, _, _ in orders.values())
    candidates = []
    for drones in range(7):
        hardware = DRONE_MASS * drones
        if parcels + hardware > truck_capacity:
            continue
        feasible = []
        for row in rows:
            if row["drones"] > drones or row["peak_drones_used"] > drones or not row["physical"]:
                continue
            metrics = row["physical"]
            added_fuel = fuel_load_rate * hardware * metrics["truck_m"] / truck_speed
            added_cost = carbon_rate * fuel_factor * added_fuel
            feasible.append(FleetPlan(drones, row["name"], row["peak_drones_used"], hardware, parcels + hardware,
                                     added_fuel, added_cost, metrics["objective"] + added_cost))
        candidates.append(min(feasible, key=lambda candidate: (candidate.operating_objective, candidate.source)))
    return candidates


def choose(plans, fee):
    return min(plans, key=lambda plan: (plan.total(fee), plan.drones))


def thresholds(plans):
    return [(low, high, (low.operating_objective - high.operating_objective) / (high.drones - low.drones))
            for low, high in itertools.combinations(plans, 2)]


def active_intervals(plans, pairs):
    cuts = sorted({0.0, *(fee for _, _, fee in pairs if fee > 0)})
    intervals = []
    for lo, hi in zip(cuts, [*cuts[1:], None]):
        fee = (lo + hi) / 2 if hi is not None else lo + 1
        plan = choose(plans, fee)
        if intervals and intervals[-1][2] == plan:
            intervals[-1] = intervals[-1][0], hi, plan
        else:
            intervals.append((lo, hi, plan))
    return intervals


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=pathlib.Path, default=OUTPUT)
    parser.add_argument("--cases", type=pathlib.Path, default=CASES)
    args = parser.parse_args()
    journal = args.output / "operating_routes_ms350_seed42.jsonl"
    with journal.open(encoding="utf-8") as stream:
        rows = [json.loads(line) for line in stream]
    cases = {}
    for row in rows:
        if row["family"] in ("reference", "fleet"):
            cases.setdefault(row["case"], []).append(row)
    assert len(cases) == 36
    assert all({row["drones"] for row in subset} == set(range(7)) for subset in cases.values())

    all_plans = []
    grid_choices = []
    pairwise = []
    switches = []
    intervals = []
    for case, subset in sorted(cases.items()):
        geometry = subset[0]["geometry"]
        common = {"case": case, "geometry": geometry}
        plans = deployment_plans(subset, args.cases / f"{case}.txt")
        intersections = thresholds(plans)
        fleet_intervals = active_intervals(plans, intersections)
        all_plans.extend({**common, "installed_drones": p.drones, "route_source": p.source,
                          "peak_drones_used": p.peak_drones_used, "equipment_mass_kg": p.hardware_mass,
                          "initial_mass_in_payload_kg": p.loaded_mass,
                          "equipment_fuel_increment_l": p.fuel_increment,
                          "equipment_carbon_charge_cny": p.carbon_cost_increment,
                          "objective_before_fee_cny": p.operating_objective} for p in plans)
        for fee in FEE_GRID:
            plan = choose(plans, fee)
            grid_choices.append({**common, "fee_per_drone_per_route_cny": fee, "selected_drones": plan.drones,
                                 "peak_drones_used": plan.peak_drones_used, "route_source": plan.source,
                                 "objective_with_fee_cny": plan.total(fee)})
        for low, high, fee in intersections:
            pairwise.append({**common, "lower_drones": low.drones, "higher_drones": high.drones,
                             "pairwise_fee_break_even_cny": fee, "larger_fleet_ever_better_at_nonnegative_fee": fee > 0})
        for start, end, plan in fleet_intervals:
            intervals.append({**common, "fee_low_cny": start, "fee_high_cny": end, "selected_drones": plan.drones,
                              "route_source": plan.source, "objective_before_fee_cny": plan.operating_objective})
        for (_, threshold, before), (_, _, after) in zip(fleet_intervals, fleet_intervals[1:]):
            assert before.drones > after.drones
            switches.append({**common, "fee_break_even_cny": threshold, "fleet_below_threshold": before.drones,
                             "fleet_above_threshold": after.drones})
    shares = []
    for geometry, fee in itertools.product(("ALL", "U", "R", "CL"), FEE_GRID):
        subset = [row for row in grid_choices if row["fee_per_drone_per_route_cny"] == fee
                  and (geometry == "ALL" or row["geometry"] == geometry)]
        for drones in range(7):
            count = sum(row["selected_drones"] == drones for row in subset)
            shares.append({"geometry": geometry, "fee_per_drone_per_route_cny": fee, "installed_drones": drones,
                           "selected_cases": count, "total_cases": len(subset), "share": count / len(subset)})
    for suffix, table in (
        ("plans", all_plans), ("grid_choices", grid_choices), ("fleet_shares", shares),
        ("pairwise_thresholds", pairwise), ("fee_intervals", intervals), ("active_switches", switches),
    ):
        write_csv(args.output / f"deployment_{suffix}.csv", table)


if __name__ == "__main__":
    main()
