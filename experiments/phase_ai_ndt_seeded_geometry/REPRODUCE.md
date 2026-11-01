# Reproduce

Python 3, NumPy, SciPy; from the independent repository root. No production source import, ROS graph, GPU matching run or production topic publication is required. Six new synthetic interface tests are independently reproducible without private data:

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 -m unittest discover -s tests -p test_seeded_geometry.py -v
```

Authorized real-data reproduction additionally requires the privately retained AH selection, complete prior records, AH original NDT results, native float64 Source files, original map_target.bin and V input contract under IESKF_DATA_ROOT. Exact hashes are recorded in manifest.json; raw records are not public. A2RL_ROOT may locate original protected records locally but is not imported or changed by these tools. No downloadable test map or real full-state payload is included.

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 experiments/phase_ai_ndt_seeded_geometry/run_frozen.py --data-root "$IESKF_DATA_ROOT" --budget 4
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 experiments/phase_ai_ndt_seeded_geometry/run_frozen.py --data-root "$IESKF_DATA_ROOT" --budget 16
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 experiments/phase_ai_ndt_seeded_geometry/posterior_diagnostic.py --data-root "$IESKF_DATA_ROOT"
```

The 16-loop command automatically selects only E1/E5 and never creates other seeds. Four-loop output reproduces historical exit reasons/endpoints but withholds all P creation; the extended output is a diagnostic, not a deployment configuration. Ignored private results preserve full states/P/trials/pair identities and permit re-evaluation. Public key_results.json contains only approved deidentified statistics. Exact CPU timing is environment dependent. Input absence must cause failure, not synthetic substitution.

The research-code commit is the enclosing Phase AI commit (`git log -1 -- experiments/phase_ai_ndt_seeded_geometry`); its parent is the explicit baseline in the manifest. This avoids embedding an impossible self-referential commit hash.

