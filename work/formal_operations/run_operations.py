import argparse
import concurrent.futures
import csv
import itertools
import json
import pathlib
import queue
import statistics
import subprocess
from dataclasses import asdict, dataclass, replace


WORK = pathlib.Path(__file__).resolve().parent
ROOT = WORK.parents[1]
CASES = ROOT / "work" / "formal_cases" / "cases"
OUTPUT = ROOT / "outputs" / "experiment-results" / "operations"
SOLVER = WORK / "solver" / "operate.exe"


@dataclass(frozen=True)
class Scenario:
    name: str
    family: str
    policy: str = "integrated"
    drones: int = 3
    capacity: float = 4.5
    battery: float = 0.510
    speed: float = 11.5
    carbon: float = 0.264
    wait: float = 0.005
    grid: float = 0.577


def make_scenarios():
    base = Scenario("reference", "reference")
    yield base
    for drones in (0, 1, 2, 4, 5, 6):
        yield replace(base, name=f"fleet_{drones}", family="fleet", drones=drones)
    for capacity in (3.5, 4.0, 5.0, 5.5):
        family = "payload" if capacity < 4.5 else "payload_stress"
        yield replace(base, name=f"payload_{capacity:.1f}", family=family, capacity=capacity)
    for reserve in (0.10, 0.20, 0.25):
        yield replace(base, name=f"reserve_{reserve:.2f}", family="reserve",
                      battery=round(0.600 * (1 - reserve), 3))
    for policy in ("single_drop", "no_wait", "constant_energy", "emissions_only"):
        yield replace(base, name=policy, family="policy", policy=policy)
    yield replace(base, name="measured_speed", family="energy", speed=13.41)
    for carbon, wait, grid in itertools.product(
        (0, 0.132, 0.264, 0.528, 1.056), (0, 0.005, 0.010, 0.020), (0.25, 0.55, 0.80)
    ):
        yield replace(base, name=f"prices_{carbon:.3f}_{wait:.3f}_{grid:.2f}", family="prices",
                      carbon=carbon, wait=wait, grid=grid / 0.93)


def selected_cases():
    return [CASES / f"n100_{geometry}_{replicate:02d}.txt"
            for geometry, replicate in itertools.product(("U", "R", "CL"), range(1, 13))]


def run_case(task, seed, budget_ms, cpu_slots):
    scenario, case = task
    cpu = cpu_slots.get()
    try:
        command = [
            str(SOLVER), str(case), scenario.policy, scenario.drones, scenario.capacity,
            scenario.battery, scenario.speed, scenario.carbon, scenario.wait, scenario.grid,
            seed, budget_ms, cpu,
        ]
        response = subprocess.run([str(arg) for arg in command], check=True, capture_output=True,
                                  text=True, timeout=15 + budget_ms / 1000)
        result = json.loads(response.stdout)
        return {"case": case.stem, "geometry": case.stem.split("_")[1], "seed": seed,
                "budget_ms": budget_ms, **asdict(scenario), **result}
    finally:
        cpu_slots.put(cpu)


def write_csv(path, rows):
    if rows:
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def report(journal, output):
    rows = list(journal.values())
    records = []
    metrics = ("objective", "operating_cost", "carbon_cost", "waiting_cost",
               "emissions_kg_co2e", "truck_m", "air_m", "fuel_l", "air_kwh", "wait_s")
    for row in rows:
        record = {key: value for key, value in row.items() if key not in ("planning", "physical", "plan")}
        record.update(physical_valid=bool(row["physical"]), planning_valid=bool(row["planning"]))
        record.update({key: (row["physical"] or {}).get(key) for key in metrics})
        records.append(record)
    records.sort(key=lambda row: (row["family"], row["name"], row["case"]))
    write_csv(output / "runs.csv", records)
    summaries = []
    for name in sorted({row["name"] for row in records}):
        for geometry in ("ALL", "U", "R", "CL"):
            group = [row for row in records
                     if row["name"] == name and (geometry == "ALL" or row["geometry"] == geometry)]
            if not group:
                continue
            valid = [row for row in group if row["physical_valid"]]
            paired = []
            for row in valid:
                if row["family"] == "prices":
                    continue
                reference = journal.get(("reference", row["case"], row["seed"]))
                if reference and reference["physical"]:
                    paired.append(row["objective"] - reference["physical"]["objective"])
            summary = {"name": name, "family": group[0]["family"], "geometry": geometry,
                       "cases": len(group), "valid": len(valid),
                       "cases_with_battery_overrun": sum(row["battery_overrun_count"] > 0 for row in group),
                       "total_battery_overruns": sum(row["battery_overrun_count"] for row in group),
                       "paired_n": len(paired), "mean_paired_cost_delta": statistics.mean(paired) if paired else None,
                       "paired_delta_sd": statistics.stdev(paired) if len(paired) > 1 else None}
            for key in (*metrics, "drone_orders", "peak_drones_used"):
                summary[f"mean_{key}"] = statistics.mean(row[key] for row in valid) if valid else None
            summaries.append(summary)
    write_csv(output / "summary.csv", summaries)

    fleets = []
    for row in records:
        if row["family"] != "fleet" and row["name"] != "reference":
            continue
        d = row["drones"]
        if d == 0 or not row["physical_valid"]:
            continue
        prior = journal.get(("reference" if d == 4 else f"fleet_{d - 1}", row["case"], row["seed"]))
        if prior and prior["physical"]:
            fleets.append({"case": row["case"], "geometry": row["geometry"], "seed": row["seed"], "drones": d,
                           "saving_from_previous_cny": prior["physical"]["objective"] - row["objective"],
                           "extra_slot_used": row["peak_drones_used"] == d,
                           "previous_drone_orders": prior["drone_orders"], "current_drone_orders": row["drone_orders"]})
    write_csv(output / "fleet_marginal.csv", fleets)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--budget-ms", type=int, default=350)
    parser.add_argument("--seeds", type=int, nargs="+", default=[42])
    parser.add_argument("--cpus", type=int, nargs="+", default=[8, 9])
    parser.add_argument("--family", choices=("all", "core", "prices", "confirmation"), default="all")
    parser.add_argument("--output", type=pathlib.Path, default=OUTPUT)
    parser.add_argument("--cases", type=pathlib.Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    seed_label = "_".join(map(str, args.seeds))
    path = args.output / f"operating_routes_ms{args.budget_ms}_seed{seed_label}.jsonl"
    journal = {}
    if path.exists():
        with path.open(encoding="utf-8") as previous:
            for line in previous:
                row = json.loads(line)
                journal[(row["name"], row["case"], row["seed"])] = row
    cpu_slots = queue.Queue()
    for cpu in args.cpus:
        cpu_slots.put(cpu)
    scenarios = list(make_scenarios())
    if args.family == "confirmation":
        scenarios = [s for s in scenarios if s.family == "policy" or s.name in ("reference", "fleet_0")]
    elif args.family != "all":
        scenarios = [s for s in scenarios if (s.family == "prices") == (args.family == "prices")]
    cases = selected_cases()
    if args.cases is not None:
        cases = [args.cases / case.name for case in cases]
    if not all(case.is_file() for case in cases):
        raise FileNotFoundError("The 36 frozen n=100 benchmark cases must exist before execution.")
    pending = []
    for index, (case, seed) in enumerate(itertools.product(cases, args.seeds)):
        offset = index % len(scenarios) if args.family == "confirmation" else 0
        for scenario in scenarios[offset:] + scenarios[:offset]:
            if (scenario.name, case.stem, seed) not in journal:
                pending.append(((scenario, case), seed))
    if args.family != "confirmation":
        pending.sort(key=lambda task: scenarios.index(task[0][0]))
    print(f"{len(pending)} remaining solves, {len(journal)} previous solves", flush=True)
    with path.open("a", encoding="utf-8") as output, concurrent.futures.ThreadPoolExecutor(
        max_workers=len(args.cpus)
    ) as executor:
        tasks = [executor.submit(run_case, task, seed, args.budget_ms, cpu_slots) for task, seed in pending]
        for index, task in enumerate(concurrent.futures.as_completed(tasks), 1):
            row = task.result()
            journal[(row["name"], row["case"], row["seed"])] = row
            output.write(json.dumps(row, separators=(",", ":")) + "\n")
            output.flush()
            if index % 36 == 0 or index == len(pending):
                print(f"Completed {index}/{len(pending)} new solves", flush=True)
                report(journal, args.output)
    report(journal, args.output)


if __name__ == "__main__":
    main()
