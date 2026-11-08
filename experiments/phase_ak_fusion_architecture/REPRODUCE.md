# Reproduction

From repository root, Python with NumPy/SciPy:

```bash
PYTHONPATH=src:experiments/phase_ak_fusion_architecture python3 -m unittest discover -s experiments/phase_ak_fusion_architecture -p 'test_*.py' -v
PYTHONPATH=src python3 experiments/phase_ak_fusion_architecture/prototype.py
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

Expected:7 new +94 existing tests pass. results.json is regenerated deterministically apart from platform floating point roundoff; use tolerances in tests, not bitwise portability. No bag/map/private trajectories/config needed.

Audit: clone https://github.com/TUMFTM/FAR-LIO.git separately; checkout ef20510316ad1f8dbf2e428e4a193f700493eccd; inspect linked files in FAR_LIO_SOURCE_AUDIT.md. No build/submodule initialization required for this source audit. Paper versions are pinned in NDT_OBSERVABILITY_AND_AMBIGUITY.md. Public historical reports are evidence reused from baseline97df23b, not newly executed real trials.

manifest.json binds parent, source hashes, prototype/test hash and fixed synthetic configuration identity. Exact committed code revision is `git log -1 --format=%H -- experiments/phase_ak_fusion_architecture/prototype.py`; hashes avoid a self-referential commit SHA. The final chat delivery records pushed research SHA and observed remote main. No credentials or machine-specific paths are needed.
