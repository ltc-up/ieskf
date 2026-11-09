# AU2 — Fixed sections and mechanism candidates

Sections1/3/9, their original AS/AT membership and ±1s padding were frozen in AU1. No estimator rerun or new GNSS alignment was selected from these results. Indices below refer to original AR frame order; full private time series and plots retain physical headers and state/P.

## Where the difference appears

Section3 padding starts at frame1298: body-lateral prior−NDT .0058m, engineering track-normal prior−GNSS .1480m. At entry1317 these are .3150m and .6212m. The entry prior−NDT consists of .2535m inherited previous posterior discrepancy plus .06145m new motion-minus-NDT displacement. By exit1405 the prior−NDT is .0513m and prior−GNSS .0817m. Thus the lead-in already develops disagreement; entry is predominantly inherited, not solely the current propagation step. Section3 is not continuously accumulating the same discrepancy until exit.

Section9 already has .2890m prior−NDT and .5569m prior−GNSS at padding start849. At entry869 prior−NDT .1385m consists of inherited .1235m plus new .01506m; its GNSS-normal difference is−.1128m. By exit897 prior−NDT .0133m while GNSS-normal difference+.3064m. Reference and axis dependence are substantial. A claim that this section necessarily creates all the error anew is unsupported. Control1 starts close to NDT (.0095m at padding1100), with a small negative NDT discrepancy at entry1120 and positive at exit1167; axis transport is especially important there.

## Measured mechanisms versus physical inference

| Candidate | Direct evidence | Qualification / missing condition |
|---|---|---|
|A inherited posterior |Entry inherited fractions approximately80%/89% in3/9; full mean histories reconstruct exactly |VERIFIED conditional reference decomposition; not proof the previous estimate is physically wrong |
|B yaw/Bias integration |Prior−NDT heading at3 entry1.450deg, reduced to1.336deg by NDT; mean new heading discrepancy +.00888deg/frame in3,−.00974 in9. Actual gyro/accel bias and held-input histories saved |Nominal changes VERIFIED; NDT/INS are not true yaw/Bias. Persistent physical gyro-bias fault NOT_VALIDATED |
|C wheel/attitude coupling |Padded windows accept640/340 wheel updates without rejections in3/9; direct wheel rear-point lateral mean−.000452/−.000921m per NDT interval. Nominal base lateral velocity mean+.978/+.483m/s |Direct correction VERIFIED; velocity is an estimated state, not evidence of slip or a forbidden zero-lateral constraint. Indirect velocity/attitude effects require controlled intervention |
|D timing/repropagation |Recorded-envelope and canonical Header results exactly match AR; no implementation phase difference detected |Numerical consistency VERIFIED. Physical relative IMU/wheel/LiDAR/GNSS delay and actual ROS scheduling NOT_VALIDATED |
|E covariance/gain |Full unit lateral response about10%; prior projected variance ≪ engineering R. Full versus scalar directional response differs only about1e−4 |VERIFIED mathematical attenuation; neither covariance calibration nor a diagnosis that R must be reduced |
|F reference artefact |GNSS versus NDT new-difference trends and even sign can differ; inherited AT delay hypotheses reverse global ordering |CONDITIONAL; no GNSS-derived tuning or correctness labels used |

Nominal base forward velocity mean45.47/28.75m/s and lateral+.978/+.483m/s in3/9; control lateral−.794m/s. Such states can influence kinematic sensitivity but are not independently measured velocity errors. Per-interval force/bias/gravity/lever components close exactly; their magnitudes cannot by themselves identify physical faults. In particular real yaw motion rotates the rear-axle lever and current lateral basis: those displacement terms are not free evidence of integration error.

The most defensible conclusion is a mix of inherited history and modest observation response, plus some new NDT-referenced disagreement in section3. A universal persistent predictor bias, wrong NDT peak, wheel slip or faulty gravity/Bias is still UNKNOWN. Short fixed-observation intervention is addressed separately; it cannot certify a rematched closed loop or true driving accuracy.
