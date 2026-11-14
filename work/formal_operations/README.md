# Replicating the operating scenarios

The package includes the frozen 36 synthetic n=100 inputs, the complete scenario grid, the C++17 operating evaluator and search, and the Python 3.10+ standard-library runner and independent evaluator.

Open a Windows Visual Studio x64 Native Tools Command Prompt in the unpacked package.

```cmd
cd solver
cl /nologo /std:c++17 /EHsc /W4 /WX /O2 /MT operate.cpp io.cpp search.cpp model.cpp exact.cpp /Fe:operate.exe
cd ..
python run_operations.py --cases cases --output runs350 --budget-ms 350 --seeds 42 --cpus 8 9
python verify_operations.py --cases cases --output runs350 --budget-ms 350 --seeds 42 --require-all
python analyze_operations.py --output runs350 --budget-ms 350 --seeds 42
python deployment.py --cases cases --output runs350
python run_operations.py --family confirmation --cases cases --output runs1500 --budget-ms 1500 --seeds 42 101 503 --cpus 8 9
python verify_operations.py --confirmation --cases cases --output runs1500 --budget-ms 1500 --seeds 42 101 503 --require-all
python analyze_operations.py --output runs1500 --budget-ms 1500 --seeds 42 101 503
```

Set `--cpus` to the logical processor indices available and reserved for these trials on your machine. Each trial uses a single pinned process. The runner appends each finished trial to its budget/seed-specific JSONL and skips existing scenario/case/seed entries on restart. Keep each run family and budget in a separate output directory. Because the stopping criterion is elapsed time, output can differ when the hardware or competing workload changes.

The operating solver adds a single-drop cardinality constraint to its isolated copy of the mixed truck–drone ALNS. The check is active during ALNS repair, exact sortie generation and complete route evaluation. The full physical evaluator checks each resulting plan independently of the planning coefficients. The independent verifier reconstructs all route costs and battery feasibility directly from the frozen cases and ordered routes.

The 350 ms matrix covers fleet size, payload, available battery energy, four policy variants, directly measured 13.41 m/s energy coefficients, and a 60-cell crossed price grid. The longer policy comparison uses the reference, truck only, single drop, no waiting cost in planning, constant power in planning, and emissions-only planning. Its uncertainty summary averages valid matched seeds within each case before a stratified bootstrap over independent cases.

With Pillow available, run `python plot_operations.py --output runs350` to regenerate Figures 4–6 from the summarized 350 ms trials.
