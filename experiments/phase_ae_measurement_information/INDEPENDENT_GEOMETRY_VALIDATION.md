# Independent geometry: what is genuinely excluded and what is not

## Identity and separation

AD G1/G2 and all full-data density endpoints used the full original Source/map: their retrospective region scores are CONTAMINATED posthoc consistency, not independent validation. No GNSS/RTK/reference pose qualified for this study. No absolute position/attitude errors are claimed.

NEW train-only A/B/C/D fits use original nominal prediction/P, never G1/G2, no favorable seed. The predeclared20m XY core/inset partition assigns1100 Source to train,772 to validation; remaining2133 lie in guard regions and are not selected based on fit outcomes. Map train/validation centers268563/130805 have actual minimum gap**7.009714m**, exceeding twice2.5m normal radius. Plane banks are fitted independently inside each map subset; actual used normal-neighbor original IDs overlap**zero**. No validation residual chose a seed, mask or hyperparameter. Cross-fit D's density selection uses only training candidates at original13 starts, not all-data roots. A global plane or map systematic error can still correlate disjoint neighborhoods; spatial exclusion is not a statistical independence proof.

## All four train-only outcomes, same validation data

All four reach interior stationary endpoints under original constraints. Raw costs under DIFFERENT density arms are not ranked. Instead, compare every endpoint on the SAME original A TRAINING energy:

|Arm|Common A train energy|Untruncated nearest-plane validation RMS(m)|252 fixed-witness validation RMS(m)|Current valid support inside original R|
|---|---:|---:|---:|---:|
|A|5330.701395|2.420789|0.555924|283/772|
|B|5332.229252|2.419151|0.556566|281/772|
|C|5370.112033|2.473592|0.559128|260/772|
|D|5336.198449|2.425198|0.548809|285/772|

Nearest-plane scores include every one of772 excluded Source points; the low~34–37% radius coverage means large untruncated tails partly represent unsupported/distant plane matches. They must not be presented as physical pose errors. Every predeclared spatial block, distances, support and residual quantiles remains in key_results.json; no bad block is removed.

The SECOND fixed-witness diagnostic was declared transparently after this coverage issue: select252 validation-only nearest valid planes at the ORIGINAL prediction within original R, freeze all identities/normals, then evaluate every endpoint WITHOUT residual clipping or further selection. It supplies a common factor set, but inherits potentially wrong original correspondence; it is neither a new fold nor ground truth. The predeclared nearest-plane metric is retained, not replaced.

## Lower numerical energy can disagree with excluded geometry

Train-only A has lower common train energy than D by5.49705, yet its fixed excluded-witness RMS is worse by0.007115m(0.555924 vs0.548809). A/B exhibit the converse disagreement under untruncated nearest-plane RMS: A lower energy, but RMS worse by0.001638m. All six pairwise comparisons are exposed, not just favorable examples. Metrics conflict, effects are small and no significance or physical accuracy rank is asserted.

Historic G1 has lower original full-data energy than G2, but its retrospective fixed-witness RMS0.471063 exceeds G2's0.445692; untruncated nearest-plane RMS2.706842 exceeds2.414619, while its support392 exceeds290. Those historical scores are NOT independent because AD used those regions. They illustrate that energy, coverage and residual rankings differ; they cannot certify G2 or invalidate G1 absolutely.

Completed posthoc scoring covers all39 density endpoints without rerunning any optimization; failed endpoints are marked unqualified. Four new train-only fits are the only legitimately excluded-geometry fits. The known synthetic plane is a separate exact observability check, not a substitute for real independent pose truth.


Evidence boundary: real-input values above are deidentified diagnostics, synthetic controls are explicitly separate, and historical AD results are reused rather than repeated. No absolute accuracy, statistical calibration or production qualification is inferred. Full Source/map/normal/state/P/trajectory records, factor identities and matrices remain in ignored local data. Proposed future model work is not an executed fix.
