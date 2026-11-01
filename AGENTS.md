# Research recording requirements

Use this independent repository for every Geometry-IESKF research phase. Never modify or commit the production repository unless the user explicitly requests it.

For every completed or failed experiment:
1. Save independent code, de-identified logs, key numerical results, failures, comparisons and a self-contained conclusion under experiments/phase_xx/. Preserve failed runs, not just successful runs.
2. Update docs/EXPERIMENT_INDEX.md with objectives, actual execution, findings, final qualifications and report links.
3. Update docs/RESEARCH_STATUS.md with verified conclusions, unresolved issues and next recommendations.
4. Create a phase manifest.json recording non-reconstructive input identities, research-code commit, configuration hash, data version and reproducibility conditions. Do not publish private payloads, sensitive configuration values or machine paths.
5. Review each staged file. Exclude original restricted production source, maps, bags, trajectories, equipment/extrinsic details, credentials, unauthorized derivatives and uncertain-origin material. Keep complete originals in ignored data/.
6. Commit reviewed public material and push main to origin. Inspect remote changes/conflicts first; never force-push or overwrite remote history. Record authentication/push failures honestly.
7. Final response must state phase, conclusions, unvalidated items, commit SHA and GitHub report URL. Do not claim a report is accessible if its commit was not pushed.

Reports must stand alone for readers without local logs. Publish only explicitly authorized, de-identified aggregate numbers; full real-data reproduction still requires separate authorized inputs. A numerical pass never implies accuracy, statistical calibration or production qualification.
