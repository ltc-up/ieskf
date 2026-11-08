# Reproduce AM

Public tests:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p test_ndt_integrity.py -v
```

Authorized inputs must match AL hashes. Private unchanged AJ CUDA service must have its required libraries/device. Full raw outputs remain ignored. Commands:

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 experiments/phase_am_ndt_integrity/run_real.py --data-root "$IESKF_DATA_ROOT" --private-output data/phase_am_ndt_integrity --public-output experiments/phase_am_ndt_integrity --ndt-service "$IESKF_NDT_SERVICE"
# If the sandbox lacks device access, retain that failure and run with authorized device access:
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 experiments/phase_am_ndt_integrity/run_gpu.py --private-output data/phase_am_ndt_integrity --public-output experiments/phase_am_ndt_integrity --ndt-service "$IESKF_NDT_SERVICE"
```

The first runner creates35 predeclared jobs and preserves the initial attempt. run_gpu measures sequential requests and seven-job batch wall time directly, computes multistart support/clusters, preserves prior attempt metadata. Re-running matching is a NEW experiment; do not overwrite an archived completed run. Public results contain deidentified aggregates only. Independent real truth is not supplied. Exact code is the enclosing AM commit (`git log -1 -- src/ndt_global/integrity.py`).
