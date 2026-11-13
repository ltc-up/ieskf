# BD-P0 frozen DIRECT7 grid

Executed13 configurations ×1468 common original source/target/seed/ROI requests =19084 CUDA matches. Max35, resolution1.5, step0.2, q50; target construction and crop rules unchanged. Sources and initial poses come from the retained original15D fullpose100 lap, not from any new variant. This first stage intentionally measures frozen-input matching, not closed-loop accuracy. GLL3/step epsilon0.01/objective epsilon0.001/max35 is an additional control; frozen AY normally uses max100.

Reference was reconstructed at native scan stamps from authorized raw GPS1/full-attitude bindings, using the existing AT transform convention and0/75/150ms time hypotheses. Reconstructed75ms positions agree with retained AT native cache to13.4 micrometres maximum (floating-time interpolation differences). Historical CSV path is missing: these aggregates use the explicit raw-valid mask, not an invented replacement CSV. Earlier state-time-interpolated diagnostic aggregates remain privately preserved and are obsolete. The new results use no reference during optimization.

|relative epsilon|step epsilon|search|converged /1468|NDT XY RMSE75 m|lateral P95 m|period2 frames|instrumented align P95 ms|
|---:|---:|---|---:|---:|---:|---:|---:|
|0|0.001|OFF|1447|0.624778|0.412082|0|3.839|
|0|0.001|mt|1468|0.590970|0.391564|0|3.297|
|0|0.01|OFF|1467|0.613207|0.406646|0|2.610|
|0|0.01|mt|1468|0.591667|0.392263|0|2.869|
|0.0001|0.001|OFF|1417|0.625390|0.412654|1|4.264|
|0.0001|0.001|mt|1460|0.594903|0.393495|0|8.572|
|0.0001|0.01|OFF|1056|0.624848|0.413615|30|4.508|
|0.0001|0.01|mt|1067|0.617278|0.399237|18|16.938|
|0.001|0.001|OFF|1433|0.625169|0.412903|0|3.800|
|0.001|0.001|mt|1468|0.592342|0.393334|0|3.925|
|0.001|0.01|OFF|1467|0.616887|0.406207|0|2.938|
|0.001|0.01|mt|1468|0.597903|0.394852|0|4.061|
|0.001|0.01|gll|1467|0.611293|0.401799|0|3.337|

Each configuration retains all poses, exit reasons, scores, gradients and per-step Hessians privately. The public JSON contains all fixed segments,0/75/150ms sensitivity, iteration and latency distributions. Costs include private trace writing, exclude map startup and extra quality call, and are NOT full-cycle realtime measurements. No ranking is based on timing from these instrumented runs.

Period2 diagnostic: at least four consecutive alternating score differences, lag-two score recurrence within10% of alternating amplitude, amplitude>1e-4, and at least six iterations. Direction reversals and repeated step-cap saturation are separate outputs. Pair counts are point-Gaussian pairs, not unique source points; changing counts is not proof of changing pair identities. The current trace does not retain pair-set fingerprints, so support-set switching is unverified. MAX_ITERATIONS alone is never labeled oscillation.

The strict relative threshold0.0001 with step epsilon0.01 substantially increases MAX_ITERATIONS and measured period2 recurrences. Tightening step epsilon can also reduce the minimum step: it is not a clean stop-only ablation. MT reduces aggregate frozen reference error relative to corresponding OFF settings, but segment9 can worsen. Neither global basin correctness nor filter accuracy follows from these aggregates. MT step epsilon0.001/objective condition disabled is selected only for further AY closed-loop testing; no production adoption is proposed.
