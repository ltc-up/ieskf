# Phase AE conclusion

**The AD objective is a smooth sampling-dependent potential, not yet a trustworthy geometric measurement model. Its Hessian must NOT be used directly as an IESKF posterior.**

1. **Density sensitivity is material.** With original normals/planes fixed, exact Target duplication lowers frozen G1 energy by1076.57 and makes gradient infinity norm≈321; nominal reoptimization shifts4.68cm. Uniform half and local dense-region half shift32.66cm and25.42cm. Geometry coordinates are never moved. B/C/D all13-start results are exposed:12/13,12/13,13/13 qualified, with changed attraction/seed memberships. Original G1 still has lower frozen energy than G2 in all4 arms; no fabricated fixed-root rank reversal.
2. **Normal refit adds a separate effect.** Same20-record rule on resampled maps changes neighborhoods, normals and validity; nominal shifts from original AD/G1 become22.11/35.35/26.37cm. These are not pure mixture-multiplicity effects.
3. **Curvature has mixed provenance.** Exact full15 chain-rule reconstruction passes. At G1 the Z profile contains plane intrinsic≈12972.9, negative residual covariance≈-1834.3, kernel/cross/normalization net≈5171.5, plus prior/chart. The explicit association net is31.7% in this algebraic grouping, ~50.7% along Y. This is NOT a partition into true versus fake information: intrinsic responsibilities also depend on sampling and signed components cancel.
4. **There is a decisive observability counterexample.** For an exact plane with ZERO surface X/Y/yaw information, the same energy shrinks X/Y sigma1m→0.0745m and yaw5°→0.6505°. Half-space duplicate density induces5.44cm tangential drift. Thus numerical curvature can encode record layout instead of independent geometric accuracy. Real sigma(Z)=0.00783m has no independent supporting calibration.
5. **Lower train objective does not ensure better excluded geometry.** Four NEW train-only fits use disjoint Source and map neighborhoods (minimum map gap7.0097m, zero shared normal neighbors). A has lower common training energy than D but worse fixed excluded-witness RMS by7.1mm. Other validation metrics rank differently; unsupported tails, initial correspondence ambiguity and lack of ground truth forbid an accuracy winner. Historic G1/G2 region scores remain explicitly contaminated.
6. **Frontend/back-end decision:** retain the smooth association research framework, but do not qualify the present energy as a trusted localization frontend or IESKF probability observation. First repair/justify the observation and map-sampling/plane-error model; keep frontend proposals separate from filter information while unresolved. No new kernel, target objective or numeric fudge was tried. Stop direct use of this Laplace posterior, not the whole geometry-residual research direction.

## Qualifications

|Item|Status|
|---|---|
|AD frozen endpoint/Hessian reproduction|PASS|
|Pure multiplicity vs plane-refit isolation|PASS|
|Signed Hessian/force reconstruction|PASS|
|Density invariance / geometric observability sanity|FAIL|
|Spatial Source/map/normal-neighborhood exclusion|PASS|
|Independent absolute pose accuracy|NOT_VALIDATED|
|Use as smooth optimization research potential|CONDITIONAL|
|Trusted localization frontend|NOT_VALIDATED|
|Direct filter posterior|FILTER_POSTERIOR_NOT_QUALIFIED|
|Closed-loop/longer replay recommendation|FAIL|
|Production candidate|FAIL|

Executed:10 NEW numeric tests;13 AD endpoint rechecks, no AD capture rerun;39 fixed-plane density fits+3 normal-refit nominal fits+4 train-only spatial fits+4 synthetic fits=50 new optimization diagnostics. All13 initializers preserved where planned. Filter covariance/state commits=0; continuous runs=0. Production and historical research inputs remain unchanged.

Self-contained evidence: [density](DENSITY_SENSITIVITY.md), [force and duplicate algebra](OBJECTIVE_FORCE_DECOMPOSITION.md), [Hessian](HESSIAN_INFORMATION_AUDIT.md), [validation](INDEPENDENT_GEOMETRY_VALIDATION.md), [posterior decision](POSTERIOR_QUALIFICATION.md), [failures](FAILURES.md), [all aggregate results](key_results.json), [reproduction](REPRODUCE.md) and [manifest](manifest.json).


Evidence boundary: real-input values above are deidentified diagnostics, synthetic controls are explicitly separate, and historical AD results are reused rather than repeated. No absolute accuracy, statistical calibration or production qualification is inferred. Full Source/map/normal/state/P/trajectory records, factor identities and matrices remain in ignored local data. Proposed future model work is not an executed fix.
