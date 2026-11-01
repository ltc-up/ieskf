# Local integration boundary

Public code does not contain production NDT, EKF, deskew, initialization, launch, map or sensor modules. A local adapter supplies authorized events, already-deskewed Sources, map points, timestamps, frame definitions and extrinsics through explicit arguments. Do not deskew a Source twice or feed a diagnostic state back into the front end.

`A2RL_ROOT` identifies an optional local authorized project when consulting original records. `IESKF_DATA_ROOT` identifies ignored private input/evidence storage. `IESKF_CONFIG` loads a local frozen research configuration; defaults are generic assumptions, not calibrated parameters. No absolute machine paths are embedded in published code.

A private JSON input contains events and frames; a private contract defines the lever arm and expected target SHA. The replay CLI accepts --target to resolve the same bytes at a portable location and --source-root for stored Sources. It verifies hashes and uses native reference timestamps. Legacy-source comparison requires an explicit index and hash binding and retains reference/coverage limitations. Offline sorting is not online delay handling.

Self-contained reproduction: synthetic smoke checks and generic mathematical modules. Not self-contained: real-data extraction, original front-end regeneration, maps, private initialization/TF, historic live arrival order, sensor trajectories, and real accuracy evaluation. These stay outside GitHub.
