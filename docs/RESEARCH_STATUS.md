# Research Status

Last repository setup: 2026-11-01.

## Established so far

- A 15D error-state research prototype was constructed and exercised in offline experiments.
- The fixed-correspondence MAP numerical behavior, 6D/15D elimination, and constrained fixed-case optimization have been investigated.
- Phase Z attributed large **candidate** map-Z velocity corrections to Z-position proposals and the prior position/velocity covariance structure.
- Phase Z did not commit candidate geometry updates on the five diagnostic frames, did not run a new continuous replay, and did not verify independent accuracy.

## Not established

- Stable dynamic association / valid geometry update acceptance.
- Calibrated posterior uncertainty or independent localization accuracy.
- Production runtime and suitability.

## Next research decision

Hold the IMU/wheel speed model, noise, map and deskew algorithm fixed. Verify actual old/new deskew source provenance; distinguish candidate `delta_vz`, accepted `delta_vz`, and the replayed full-state `v_z`. Diagnose whether deskew correction reduces real accepted Z-velocity spikes under comparable replay conditions.

## Repository boundaries

No embedded Git repository under auto-eagle, and no production source mutations as part of research migration. Public repo: review every report, script and data file before publishing.
