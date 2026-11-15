# BF1 measured cycle performance — NO-GO50ms

SequentialGPUruns on the actualhost; noNDT/GICPconcurrency. Both use identicalNDTsettings. Maincomparison is B0_run1 andB1_fixed_arrival; physical timing and controlledsensoravailability are separate. Reusedservice startup355/256ms-class observations are not trackingcyclecost; exactvalues areinBF1_RESULTS. The first trackingframe includes~76–79ms ofcatch-up through mapping-periodIMU; it remains inall57statistics. Steady laterMAX_ITERATIONSframes also exceed50ms, so failingqualification does not rely onlyonstartup.

ValuesP50/P95/P99/max inms:

| Component | LegacyB0 | NativeB1commonarrival |
|---|---|---|
| Implemented full cycle | 49.539/62.091/95.959/138.387 | 49.019/63.644/91.956/126.726 |
| CUDA NDT align wall | 39.399/52.673/53.028/53.124 | 38.491/54.036/54.783/54.790 |
| Service inclIPC/quality | 40.369/53.724/54.068/54.177 | 39.588/55.045/55.876/55.902 |
| Raw read/transform/write | 1.904/2.264/2.903/3.490 | 1.967/2.625/3.227/3.504 |
| Rollback/replay | 1.114/1.288/2.869/4.669 | 1.107/1.252/1.613/1.802 |

Fullcycleover50ms: 27/57 and21/57. IncludesIMUprior, actualrawXYZIread/transform, sharedcurrent-sourcewrite, IPC, CUDAalign (includingexistingFitness/nativequality), extraqualityquery, both frozenRdiagnostics, AShadow, delayedcommit/replay andmeasurementrecordserialization/write. Timerprofile's ownsmallwrite and anyOSfsyncdurability are not included. No fixedGPUkernel-onlytime is substituted forfullcycle. Standalone covariance-transport/PCLFitness timing isnotinstrumented; no inventedbreakdown.

No additionalexactDeskew (missingperpointtimecontract), ROSexecutor/networkdelivery/controlcallback orGPUresourcecompetition qualification. Pipelineis anofflineinstrumentedfunctionalcycle; cannotclaimfullproductionend-to-endcompliance. Measuredarrival simulation usescomplete scanend plusprocessing; primarycommon-arrivalcomparison usesB0availability table forbothmodes. B1would miss the imposedtable in31/57measuredcomputeinstances. It iscounterfactualdelaycontrol, notproof ofreal-timedeadlineachievement.

The few-msbetween-run differences are not attributable to covariancechoice: matcherperforms samework andboot/cache/clockjitter differ. No performanceoptimization, pointdiscarding orparameterchange was applied. T7 is notused. Diskread/transform is explicitly measured; cachecoldness/bandwidth limits were not independentlybenchmarked.
