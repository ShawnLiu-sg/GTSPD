# Truck–drone model validation

The arc MILP implements the manuscript's single-truck, same-stop model with explicit binary launch and delivery indicators and truck and drone payload balances. The exact enumeration uses a separate forward evaluator and searches all truck subsets and orders, all admissible sortie orders, and their disjoint allocations. The heuristic smoke check uses the same generated tiny cases as the MILP validation.

## Build and run

Use MSVC 2019 and Gurobi 12.0.1 with a valid local license. The source archive contains this folder, the companion `compare` sources, and the 36 frozen S1 case files under `s1_cases`. Use the Developer Command Prompt from this folder.

```bat
cl /nologo /std:c++17 /W4 /WX /EHsc /O2 /I "%GUROBI_HOME%\include" main.cpp oracle.cpp milp.cpp ..\compare\io.cpp ..\compare\search.cpp ..\compare\model.cpp ..\compare\exact.cpp /Fe:model_validation.exe /link "%GUROBI_HOME%\lib\gurobi120.lib"
set "PATH=%GUROBI_HOME%\bin;%PATH%"
model_validation.exe tiny tiny_exact_validation.csv
model_validation.exe tiny-search tiny_search_validation.csv
model_validation.exe local local_exact_validation.csv
model_validation.exe s1 s1_warm_base_full.csv ..\s1_cases 10 warm
model_validation.exe s1 s1_cold_base_full.csv ..\s1_cases 10 cold
model_validation.exe s1 s1_cut_families.csv ..\s1_cases 10 families
model_validation.exe s1-lp s1_extended_lp.csv ..\s1_cases 30
```

`warm` uses all 36 S1 instances. `cold` and `families` use case IDs `01` and `02` in each of the six size-by-geometry strata. The separate 30 s LP diagnostic uses ID `01` in each stratum. Tiny generation uses seed 410936 and constructs 24 instances, six per customer count from 5 to 8. Cases include no drones, no admissible launch stop, zero carbon and dwell prices, exact truck and aircraft payload boundaries, directed metrics and insufficient flight energy. The search smoke uses one 100 ms run per case with seed 410936 and records the returned plan and reference plan. Local validation uses another 24 deterministic fixed-tour neighborhoods with one or two selected stops. It compares the released exact DP with direct exhaustive partition evaluation and a separately built Gurobi set-partitioning model over all feasible ordered patterns. It retains full plans, sizes and timings, and includes exact duration ties, an unused stop, a launch restriction and a controlled carriage–air cost trade-off. Search and model validation use the same operational cost components. Tiny scenario parameters are controlled test inputs. The raw tiny and local CSV files are included under `tiny_results`.

## Cut implementation

- **Packing.** Solve the geometry- and battery-free packing relaxation for up to 0.3 s. Add the certified integer lower bound on truck stops. The simple mandatory-customer lower bound supplies a fallback.
- **Symmetry.** Sort the first delivered node IDs of the drone sorties at each stop in descending order. Unused drones have key zero.
- **Capacity covers.** Generate minimal covers from cyclic seeds in customer index order. Apply exact sequential lifting with a support cap of 14 terms per cut. The seed enumeration and support cap define the implemented family.
- **Battery and conflict graph.** Exclude unreachable launch–customer pairs, add incompatible-pair constraints from payload or the minimum energy of the two direct delivery orders, and construct greedy conflict cliques using degree and customer index. The subset-size-two pair test uses exact two-delivery flight energy. Larger clique cuts represent pairwise conflicts.

## Matched timing and CSV

Run one Gurobi thread and pin the process to logical CPU 10 when available. The tiny heuristic smoke and local checks use logical CPU 11. MIP timing includes environment and model creation, packing and cut generation, and the warm-start installation and solve. Before each matched pair, generate a common nearest-neighbour plus 2-opt all-truck incumbent. Cold variants omit the MIP start. Use relative MIP gap 0, absolute MIP gap `1e-8`, and integrality and feasibility tolerances `1e-9`. A decoded plan is independently evaluated before recording it, using a `1e-6` objective consistency threshold.

`grb_status=2` is certified optimal and `9` is time limited. `root_callback_bound` is the best root-node bound observed from the MIP callback when available. `lp_relaxation_objective` is from a separate solve of the built formulation with integrality relaxed. It is reported only when `lp_relaxation_status=2`. This LP diagnostic is timed separately and does not change either solver variant's MIP budget. Blank result cells indicate values unavailable at termination. Each completed variant is flushed to CSV.
