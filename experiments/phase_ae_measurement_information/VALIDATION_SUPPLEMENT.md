# Additional evaluation diagnostics after initial fold coverage check

The predeclared UNTRUNCATED nearest-valid-plane validation remains unchanged. It finds only about one third of validation Source inside the original1m radius; distant-plane residual tails also reflect missing support. Such tails must not be presented as reliable physical pose error.

Before computing any supplementary scores, add a SECOND witness fixed at the ORIGINAL prediction: for each predeclared validation Source use its validation-only nearest valid plane, retain all initially within the original1m radius. Freeze those Target/normal identities, then score ALL endpoints without residual truncation or further selection, including distances/support changes. This isolates a common frozen geometric factor set but inherits potentially wrong initial correspondence; it is NOT absolute truth and does not replace the original metric.

Also evaluate every train-only fit on the SAME original A training energy before comparing train fit versus held-out consistency. Raw objective values from different density variants are NOT comparable as accuracy. Complete the missing full-data posthoc scores after density runs finish; do not rerun the four training fits or change the fold. Historic full-data fits remain contaminated for validation claims.
