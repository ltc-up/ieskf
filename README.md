# Geometry-IESKF Research

Independent research workspace for a 15-state geometry-aided inertial error-state Kalman filter (ESKF).

## Repository layout

- `src/` — independent research implementation (to be migrated locally).
- `experiments/` — versioned research phases and reproducibility notes.
- `docs/EXPERIMENT_INDEX.md` — phase summary and result links.
- `docs/RESEARCH_STATUS.md` — current research gates and open questions.
- `scripts/` and `tests/` — reproducibility tooling and independent tests.
- `configs/` — research-only configuration.
- `data/` — **local-only** experimental inputs (Git-ignored).

This repository is separate from the production A2RL / auto-eagle source tree. Research outcomes must not be presented as production-ready until independent accuracy, statistical consistency, and runtime criteria have been met.

## Local setup

```bash
cd /home/ltc/flyagle
git clone https://github.com/ltc-up/ieskf.git
cd ieskf
```

Set `A2RL_ROOT` and `IESKF_DATA_ROOT` explicitly for local-only dependencies. Do not copy the production repository's `.git` or publish bag/PCD/raw sensor files.

## Publishing policy

The repository is **public**. Before committing a report, confirm ownership and permission to publish any code, experimental data, maps, derived artifacts, internal paths, and third-party material. Add reproducible code and redacted reports only; large or restricted data should be referenced by local manifest and hashes.

## Workflow

Each phase adds a final report, a reproducibility note, a lightweight input identity manifest, and optional scripts. Update `docs/EXPERIMENT_INDEX.md` and `docs/RESEARCH_STATUS.md` at the same commit.
