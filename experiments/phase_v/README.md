# Phase V: Full inertial error-state prototype

Propagate position, world velocity, SO(3) attitude and gyro/acceleration Bias with complete cross covariance. Add a longitudinal wheel observation and iterated map factors using one frozen prediction prior. Conditional IRLS covariance is transported once. Offline operation was demonstrated, but iteration-limit partial acceptance and dynamic association prevent reliable deployment/uncertainty claims.

The original test plan, source, parameters, full evidence, failure logs and input hashes are retained unchanged in ignored `data/history/phase_v/`. This is a separate public redaction; it contains no real sensor results. Portable independent modules and transformation hashes are listed in `docs/CODE_PROVENANCE.json`. Private extraction/driver/configuration dependencies remain local and are not independently reproducible from this repository.
