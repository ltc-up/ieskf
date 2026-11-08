# Actual controlled injections
AO commit/configuration/core hashes are frozen. The entire same loop44 trace/observation stream is the no-fault control. Conditions were declared before AP execution and no noise/threshold/model was changed.

| Scenario | Actual alteration | Result |
|---|---|---|
| control | none |16316 NDT UNKNOWN,9170 GNSS unqualified;0 anchors |
| GNSS dropout | middle third removes3056 messages |6114 remaining unqualified; no qualified recovery test |
| GNSS delay |9170 receipts delayed600ms |9170 STALE |
| GNSS outlier |9170 raw latitude fields +0.001degree |qualification blocks conversion; numerical outlier test NOT_EXERCISED |
| wrong first NDT | first candidate +100m X/+50m Z |manager UNKNOWN, no anchor; shadow NO_BRACKET at fixed startup frame |
| duplicate first NDT | one duplicate correlation identity |one DUPLICATE; no repeated anchor |
| delayed first NDT | one receipt +600ms |OBS_TIME_ROLLBACK because newer observations already arrived; reason priority retained |
| ambiguous first NDT | explicitly supplied second hypothesis10m away |one AMBIGUOUS_NO_ANCHOR;40 AMBIGUOUS output slots |
| NDT absence | middle third removes5265 NDT observations |11051 remain UNKNOWN; not “no raw LiDAR cloud” |

Every case uses91707 output slots,0 trusted anchors and917.07s without trusted localization. No runtime local-state mutation is possible through the read-only interface; full trace hashes agree. This is a software-boundary test, not real wrong-peak recognition or a successful localization system.

The fixed startup wrong-candidate shadow probe fails its history prerequisite; it was not moved to another frame. First full run is retained in `fault_attempt1.json`; a second run adds the previously missing rejection-reason instrumentation, with unchanged core/config/conditions. Synthetic bracket-valid unit coverage separately proves that shadow publication can be wrong and still isolate local state. That synthetic result does not replace the failed real-input probe.

The20m Target deletion is a separate artificial geometry injection on real frozen clouds. Partial-LiDAR Source rebuilding was not performed; random fused-point deletion was not used. Real and injected evidence remain distinct throughout.
