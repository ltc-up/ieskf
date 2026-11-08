# Frozen baseline
AO commit: **3667c9c6da84b82c1d9e4676b38436ed9f8f6da1**, pushed and remote SHA verified before AP. Full AO qualification is NO-GO. AP does not inherit a normal-lap accuracy/availability PASS.

`frozen_identity.json` records SHA256 for every existing research source and AO artifact plus unchanged configuration identity. `test_exact_AO_code_and_config_frozen` verifies those bytes. New AP runners/tests/reports are additions only. No existing predictor, wheel model, global manager, shadow corrector, diagnostic thresholds, GNSS qualification, Q/R, state definitions or data interface changed.

Real raw/local identities match AO loop44; the whole 917.07 s produced trace is reused read-only. Identical trace SHA before/after all nine global fault scenarios proves no mutation of the executed local trajectory. It does not validate the local trajectory, which already fails height behavior. Historical five-group Source/Target/multistart diagnostics remain separately historical; they are not fresh full-lap matching output.
