# Map error evidence

No real Sigma_plane entry is calibrated in AG. All conditional solves reuse AF's ONE transferred synthetic covariance law, recorded unchanged in code/input hashes. It is not promoted to a map sensor specification or production default.

|Quantity|Available evidence|Status|
|---|---|---|
|TLS normal/offset fit|one stored map and declared grouping|geometric estimate only|
|Normal/offset sampling covariance|no qualified independent reconstructions or true plane references|UNIDENTIFIED|
|Cross-patch common error|same-surface identities unresolved|UNIDENTIFIED|
|Common map vertical/ENU datum error|no datum/registration covariance chain|UNIDENTIFIED|
|Map vs Source/prior dependence|mapping scan membership/localization chain missing|UNKNOWN|

AF baseline fit RMS median0.267m/P90 0.444m exceeds the transferred synthetic offset std0.1m in many groups. Neither this scatter nor a TLS Hessian is a calibrated plane parameter variance; nonplanarity, outliers, sampling bias and common pose error are mixed. Within-map Target bootstrap would not recover coherent datum/trajectory or scan-pose errors and was NOT performed. No map re-fit, Q/R tuning or threshold selection in AG.

A legal5m optimization endpoint later has conditional GN sigmaZ0.03265m under this hypothesis. That is a conditional curvature diagnostic, NOT evidence that true Z uncertainty is3.3cm. Model assumptions, independence and segmentation dominate its interpretation.
