# Test evidence

Final publication suite: **218 passed**. Existing motion/Joseph/reset/optical tests remain in the suite; new tests exercise coupled physical information charts, synthetic weak-longitudinal/strong-lateral retention, branch independence, normalized and deduplicated frame scores, terminal IDs, expiry, exact state/full-P rollback, cascade rejection, legacy replay equality, no disjoint-pose averaging, switching hysteresis, versioned path-frame contracts, GNSS-off source blocking and asynchronous cloud buffer completion.

The actual production queue header was compiled into the private compatibility harness and passed. It demonstrates five pop/age attempts and persistence of old observations after refusing a new one; it is not a production estimator replay.

A final integrated 30-frame CUDA/dual-branch run exercises the final code with injected refusal and historical revisions. It is a smoke regression, not a replacement for long-run results. The selected long-run revised versions are separately SHA-recovered under source_versions/.

Preserved negative test history: the initial whole-repository collection traversed the authorized data symlink and failed on upstream ROS dependencies; the corrected test root is tests/. A later comparison-switch edit introduced a syntax error before scans started, and two tests failed (an error-message expectation and a test modifying a returned copy). Startup syntax, actual replay state mutation and conservative merge semantics were corrected before the controlled2 reruns. The tests then passed216,217 and finally218 as new independent geometry/cascade regressions were added. Original logs remain private; no failed startup is counted as a processed trajectory.

Numerical tests establish the stated event/model contracts only. They do not certify true mode labels, calibrated covariance, physical timing/installation, hard real-time behavior or a safe vehicle controller.
