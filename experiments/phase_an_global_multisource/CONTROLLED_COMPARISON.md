# Same-window real comparison — strict qualification FAIL

All groups use the original14.700655404s native window with284 frozen Sources,matching event identity and AL/AM Target digest. Local/no-map data were freshly replayed in AL; AN consumes those immutable results and adds actual146 raw GNSS records plus five AM same-Source multistart batches. No future branch result drives local prediction. Stored AJ seeds/Deskew remain correlated historical frontend inputs,not a newly generated AN causal frontend.

| Group | Actual implementation/input | Observed result | Qualification |
|---|---|---|---|
| A |1470 recorded production kinematic samples,Header map/base_link | Available historical output; per-NDT acceptance not logged | Runtime NDT enable,map-version/reference calibration unknown; not strict controlled truth |
| B |AL155 shadow corrections,129 nonconvergence rejects |284 untrusted publications; max correction-induced same-time global jump12.645m | Time composition passes; global correctness unqualified |
| C |AN actual source manager on284 NDT groups +146 GNSS messages |0 coordinate anchors,0 trusted publications | Qualified real fusion/recovery FAIL; hypotheses/health management executes |

C NDT dispositions:2 AMBIGUOUS,155 UNKNOWN,127 NOT_CONVERGED. Compared to AL,AM new batch evidence changes two formerly nonconverged groups into ambiguity evidence; the5 batches do not add independent observations. All146 GNSS inputs are received but unqualified,not missing. C output slots1470:19 INITIALIZING,1355 UNAVAILABLE,96 AMBIGUOUS. Ambiguity state occupancy6.53%; this is NOT wrong-match prevalence. Qualified global availability0%; observation anchoring rate0%; false trusted publications0 is achieved by abstention,not successful localization. AL shadow-acceptance54.58% is not trusted availability.

Local1470 state/P slots remain bitwise equal to no-map control. Real local Z span27.552m,maximum wheel-induced dZ4.17699m/dvz.550877m/s,final model sigmaZ51.7827m. These are actual inherited local-model behaviors,not independently measured height errors. Layering isolates maps but does NOT repair wheel/Z coupling or missing vertical observability.

Original map-hole ground truth,low-fitness wrong-peak labels and reduced-LiDAR real trials are unavailable. AM intervention detects support change,not validated original map-hole detection. Frozen records list3 clouds; per-point contribution attribution absent. No paired accuracy improvement,RMSE or real GNSS mode-selection/recovery success can be reported. Strict A/B/C map/runtime/control qualification fails; this is an explicitly limited same-window consistency/availability comparison,not substituted synthetic evidence.

AN core median0.00185ms,p950.00405ms,max0.07675ms on offline dispatch; excludes parsing,geometry,CUDA,Deskew,serialization and live scheduling. No end-to-end20Hz qualification. See AM actual cold/warm GPU costs.
