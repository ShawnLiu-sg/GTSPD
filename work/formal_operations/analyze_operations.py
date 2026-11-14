import argparse
import collections
import json
import math
import pathlib
import random
import statistics

from run_operations import OUTPUT, write_csv


METRICS = ("objective", "emissions_kg_co2e", "truck_m", "air_m", "air_kwh", "wait_s")


def percentiles(values, rng, repetitions=5000):
    if len(values) < 2:
        return None, None
    strata = collections.defaultdict(list)
    for geometry, value in values:
        strata[geometry].append(value)
    draws = []
    for _ in range(repetitions):
        sample = [rng.choice(group) for group in strata.values() for _ in group]
        draws.append(sum(sample) / len(sample))
    draws.sort()
    return draws[int(0.025 * (repetitions - 1))], draws[math.ceil(0.975 * (repetitions - 1))]


def matched_row(group, name, comparison, metric, values, rng):
    lo, hi = percentiles(values, rng)
    diffs = [value for _, value in values]
    return {
        "geometry": group, "name": name, "comparison": comparison, "metric": metric,
        "paired_n": len(diffs),
        "mean": statistics.mean(diffs) if diffs else None,
        "median": statistics.median(diffs) if diffs else None,
        "p2_5": lo, "p97_5": hi,
        "negative_n": sum(value < -1e-9 for value in diffs),
        "zero_n": sum(abs(value) <= 1e-9 for value in diffs),
        "positive_n": sum(value > 1e-9 for value in diffs),
    }


def energy_provenance(output):
    payload = (0.0, 1.13, 2.27, 4.54)
    source_671 = (1052.5789, 1232.8998, 1401.8190, 1878.1490)
    source_1341 = (1186.5048, 1479.2276, 1718.7145, 2044.0478)
    observed = []
    fitted = []
    for index, q in enumerate(payload):
        w1 = source_671[index]
        w2 = source_1341[index]
        observed.append({"payload_kg": q, "forward_w_6_71_ms": w1, "forward_w_13_41_ms": w2,
                         "interpolated_w_11_5_ms": w1 + (w2 - w1) * ((11.5 - 6.71) / (13.41 - 6.71))})
    for speed, key, kind in (
        (6.71, "forward_w_6_71_ms", "published_condition_means"),
        (11.5, "interpolated_w_11_5_ms", "speed_interpolated_condition_means"),
        (13.41, "forward_w_13_41_ms", "published_condition_means"),
    ):
        y = [row[key] for row in observed]
        slope, intercept = statistics.linear_regression(payload, y)
        error = [(intercept + slope * q) - w for q, w in zip(payload, y)]
        mse = statistics.mean(value * value for value in error)
        fitted.append({"speed_m_s": speed, "source_type": kind, "points": len(y), "intercept_w": intercept,
                       "slope_w_kg": slope, "in_sample_rmse_w": math.sqrt(mse),
                       "in_sample_r2": 1 - mse / statistics.pvariance(y),
                       "empty_wh_per_km": intercept / (3.6 * speed),
                       "4_5kg_wh_per_km": (intercept + 4.5 * slope) / (3.6 * speed)})
    write_csv(output / "energy_condition_means.csv", observed)
    write_csv(output / "energy_fit.csv", fitted)


def repriced_cost(value, scenario):
    emissions = 3.24 * value["fuel_l"] + scenario["grid"] * value["air_kwh"]
    objective = value["operating_cost"] + scenario["carbon"] * emissions + scenario["wait"] * value["wait_s"]
    return objective, emissions


def operations_tables(rows, output):
    records = {(row["name"], row["case"], row["seed"]): row for row in rows}
    rng = random.Random(20261113)
    matching = []
    pricing = []
    for name in sorted({row["name"] for row in rows}):
        variants = [row for row in rows if row["name"] == name]
        family = variants[0]["family"]
        if family == "reference":
            continue
        for group in ("ALL", "U", "R", "CL"):
            variants_by_group = [row for row in variants if group == "ALL" or row["geometry"] == group]
            if not variants_by_group:
                continue
            if family == "prices":
                first = variants_by_group[0]
                good = [r for r in variants_by_group if r["physical"]]
                savings = []
                for row in good:
                    baseline = records.get(("reference", row["case"], row["seed"]))
                    if baseline and baseline["physical"]:
                        repriced, _ = repriced_cost(baseline["physical"], row)
                        savings.append(repriced - row["physical"]["objective"])
                pricing.append({"name": name, "geometry": group, "carbon_cny_kg": first["carbon"],
                                "wait_cny_s": first["wait"], "grid_battery_kg_kwh": first["grid"],
                                "planned_cases": len(variants_by_group), "valid_cases": len(good),
                                "drone_used_cases": sum(r["drone_orders"] > 0 for r in good),
                                "mean_drone_orders": statistics.mean(r["drone_orders"] for r in good) if good else None,
                                "repriced_reference_paired_n": len(savings),
                                "mean_cost_saving_vs_repriced_reference_cny": statistics.mean(savings) if savings else None,
                                "paired_cheaper_cases": sum(saving > 1e-9 for saving in savings),
                                **{f"mean_{metric}": statistics.mean(r["physical"][metric] for r in good)
                                   if good else None for metric in METRICS}})
                continue
            for metric in METRICS:
                differences = collections.defaultdict(list)
                paired_runs = 0
                for row in variants_by_group:
                    ref = records.get(("reference", row["case"], row["seed"]))
                    if row["physical"] and ref and ref["physical"]:
                        differences[(row["geometry"], row["case"])].append(
                            row["physical"][metric] - ref["physical"][metric])
                        paired_runs += 1
                values = [(geometry, statistics.mean(values)) for (geometry, _), values in differences.items()]
                matching.append({**matched_row(group, name, "variant_minus_reference", metric, values, rng),
                                 "matched_feasible_runs": paired_runs, "variant_attempts": len(variants_by_group)})
    for drones in range(1, 7):
        name = "reference" if drones == 3 else f"fleet_{drones}"
        previous = "reference" if drones == 4 else f"fleet_{drones - 1}"
        for group in ("ALL", "U", "R", "CL"):
            differences = collections.defaultdict(list)
            attempts = 0
            paired_runs = 0
            for row in rows:
                if row["name"] != name or (group != "ALL" and row["geometry"] != group):
                    continue
                attempts += 1
                prior = records.get((previous, row["case"], row["seed"]))
                if prior and prior["physical"] and row["physical"]:
                    differences[(row["geometry"], row["case"])].append(
                        prior["physical"]["objective"] - row["physical"]["objective"])
                    paired_runs += 1
            values = [(geometry, statistics.mean(values)) for (geometry, _), values in differences.items()]
            if attempts and paired_runs:
                matching.append({**matched_row(group, f"D{drones}", "previous_minus_current", "objective", values, rng),
                                 "matched_feasible_runs": paired_runs, "variant_attempts": attempts})
    write_csv(output / "paired_effects.csv", matching)
    write_csv(output / "price_grid.csv", pricing)

    envelopes = []
    for case, seed in sorted({(row["case"], row["seed"]) for row in rows}):
        if ("fleet_6", case, seed) not in records:
            continue
        incumbent = None
        previous = None
        for drones in range(7):
            name = "reference" if drones == 3 else f"fleet_{drones}"
            row = records.get((name, case, seed))
            if row and row["physical"]:
                if incumbent is None or row["physical"]["objective"] < incumbent["physical"]["objective"]:
                    incumbent = row
            if incumbent:
                value = incumbent["physical"]["objective"]
                envelopes.append({"case": case, "geometry": incumbent["geometry"], "seed": seed, "drones": drones,
                                  "best_known_cost_cny": value, "plan_source": incumbent["name"],
                                  "best_known_cost_improvement_cny": previous - value if previous is not None else None,
                                  "peak_drones_used": incumbent["peak_drones_used"]})
                previous = value
    write_csv(output / "fleet_feasible_envelope.csv", envelopes)

    price_pool_rows = []
    price_pool_summary = []
    price_names = sorted({row["name"] for row in rows if row["family"] == "prices"})
    price_pools = collections.defaultdict(list)
    for row in rows:
        if row["family"] in ("reference", "prices") and row["physical"]:
            price_pools[(row["case"], row["seed"])].append(row)
    for name in price_names:
        chosen = []
        for (case, seed), pool in price_pools.items():
            scenario = records.get((name, case, seed))
            baseline = records.get(("reference", case, seed))
            if not scenario or not baseline or not baseline["physical"]:
                continue
            incumbent = min(pool, key=lambda row: (repriced_cost(row["physical"], scenario)[0], row["name"]))
            cost, emissions = repriced_cost(incumbent["physical"], scenario)
            baseline_cost, _ = repriced_cost(baseline["physical"], scenario)
            chosen.append({"name": name, "case": case, "seed": seed, "geometry": scenario["geometry"],
                           "selected_source": incumbent["name"], "n_candidates": len(pool),
                           "objective": cost, "emissions_kg_co2e": emissions,
                           "drone_orders": incumbent["drone_orders"], "air_kwh": incumbent["physical"]["air_kwh"],
                           "truck_m": incumbent["physical"]["truck_m"], "wait_s": incumbent["physical"]["wait_s"],
                           "saving_vs_repriced_reference": baseline_cost - cost})
        price_pool_rows.extend(chosen)
        for geometry in ("ALL", "U", "R", "CL"):
            group = [row for row in chosen if geometry == "ALL" or row["geometry"] == geometry]
            if group:
                metrics = ("objective", "emissions_kg_co2e", "drone_orders", "air_kwh", "truck_m", "wait_s",
                           "saving_vs_repriced_reference")
                price_pool_summary.append({"name": name, "geometry": geometry, "cases": len(group),
                                           **{f"mean_{k}": statistics.mean(row[k] for row in group) for k in metrics}})
    write_csv(output / "price_candidate_pool.csv", price_pool_rows)
    write_csv(output / "price_candidate_pool_summary.csv", price_pool_summary)

    policies = []
    for name in sorted({row["name"] for row in rows if row["family"] == "policy"}):
        attempts = [row for row in rows if row["name"] == name]
        valid_pairs = [(row, records.get(("reference", row["case"], row["seed"]))) for row in attempts]
        valid_pairs = [(row, ref) for row, ref in valid_pairs if row["physical"] and ref and ref["physical"]]
        diff = [row["physical"]["emissions_kg_co2e"] - ref["physical"]["emissions_kg_co2e"]
                for row, ref in valid_pairs]
        policies.append({"policy": name, "attempts": len(attempts),
                         "no_incumbent": sum(not row["planning"] for row in attempts),
                         "battery_invalid": sum(row["battery_overrun_count"] > 0 for row in attempts),
                         "paired_feasible_runs": len(valid_pairs), "higher_emission_runs": sum(d > 1e-9 for d in diff),
                         "equal_emission_runs": sum(abs(d) <= 1e-9 for d in diff),
                         "lower_emission_runs": sum(d < -1e-9 for d in diff)})
    write_csv(output / "policy_pair_diagnostics.csv", policies)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--budget-ms", type=int, default=350)
    parser.add_argument("--seeds", type=int, nargs="+", default=[42])
    parser.add_argument("--output", type=pathlib.Path, default=OUTPUT)
    args = parser.parse_args()
    energy_provenance(args.output)
    seed_label = "_".join(map(str, args.seeds))
    with (args.output / f"operating_routes_ms{args.budget_ms}_seed{seed_label}.jsonl").open(encoding="utf-8") as data:
        rows = [json.loads(line) for line in data]
    operations_tables(rows, args.output)


if __name__ == "__main__":
    main()
