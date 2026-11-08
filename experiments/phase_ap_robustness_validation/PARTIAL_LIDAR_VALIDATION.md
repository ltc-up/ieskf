# Partial LiDAR qualification
| Input combination | AP status |
|---|---|
| front+left+right | Real raw topics and historical fused Sources exist; fresh whole-lap frontend/registration NOT_VALIDATED |
| front only | NOT_VALIDATED |
| front+left | NOT_VALIDATED |
| front+right | NOT_VALIDATED |
| left+right | NOT_VALIDATED |
| one source short outage | NOT_VALIDATED |

No random deletion from a fused cloud was relabeled a LiDAR failure. The recorded fused Source no longer supplies adequate per-point origin/provenance to reconstruct these combinations, and a qualified whole-lap original per-source Deskew/selection/causal motion chain was not completed. Raw availability counts alone cannot establish that the production processor accepts, synchronizes or publishes each subset.

Existing AL candidate `lidars`/correlation-group interfaces can carry source availability, but unknown fields remain UNKNOWN. Source absence, stale cloud, unavailable supported geometry, nonconvergence and observation rejection must remain distinct. The NDT-middle-third injection is **NDT-observation absence**, not a claim that raw LiDAR had no point clouds. No single-/dual-LiDAR matching success rate is reported. This requested engineering validation remains a blocker.
