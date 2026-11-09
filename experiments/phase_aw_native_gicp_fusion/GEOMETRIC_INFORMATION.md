# Coupled shape comparison, not noise calibration

Unified chart is `[body translation(m),10*body-right rotation(rad)]` at the base reference. NDT internal-map/Euler and GICP right-twist charts are transported by full congruences. Retain matrix eigenvectors, eigenvalues, cross blocks and conditional-axis energy. For translation marginal information use Schur complement `Itt−Itr Irr⁻¹Irt`; conditional diagonal is not an independent direction covariance. Relative shape is normalized by trace; no comparison of raw objective scores or raw cross-algorithm eigenvalue magnitude.

1468 actual same-input matches: full-matrix shape cosine median0.900, weakest-vector absolute cosine median0.447 (range≈0..1). Broad anisotropic structure agrees more than individual weakest directions. NDT and GICP do NOT always identify the same weak pose combination. This is consistent with voxel-best-neighbor/inlier modelling versus planar source+target local covariance and correspondence changes; it is not proof one matcher is geometrically correct.

|Fixed segment|NDT marginal lateral share|GICP marginal lateral share|NDT lateral–Yaw normalized coupling|GICP coupling|
|---|---:|---:|---:|---:|
|0|0.407|0.597|0.096|0.017|
|1|0.337|0.353|0.022|0.143|
|3|0.396|0.841|0.024|0.197|
|5|0.331|0.143|0.121|0.093|
|8|0.282|0.174|−0.331|−0.165|
|9|0.353|0.363|0.112|0.607|

Full normalized matrices/eigenvectors remain private; de-identified corner aggregates are public. A high lateral share is relative to other directions, not a small lateral error bar. Right9's strong GICP lateral–Yaw coupling shows why diagonal scaling loses meaningful local structure. It does not establish a good statisticalR.

Marginal lateral shape versus absolute engineering lateral error Spearman: NDT≈+0.051, GICP≈−0.055 on the common valid reference interval. No reliable monotone error/noise mapping is supported. Adjacent frames and repeated track locations are correlated; these are descriptive associations, not detection precision/recall or calibration.

Six preselected segment-midpoint cases expose final-factor fixed-correspondence cost±axis perturbations. Numerical gradient uses the frozen Mahalanobis factors; GN curvature can differ from true nonlinear frozen-factor curvature by residual second-order terms. Real rematching changes correspondences/weights, so these trials do not bound global pose error. Public aggregate probes retain actual differences. Multiple local solutions up to0.319m for GICP in segment5 and0.152m in3 are observed under fixed±seed probes; independent correct-peak labels are absent. Full rank/low residual/consistent probes are never treated as a correctness certificate.

Reused [X-ICP](https://arxiv.org/html/2211.16335v3) §V information analysis and pair-contribution categorization distinguish support from eigenvalues; §VI applies controlled optimization constraints rather than inventing measurementR. AW uses its diagnostic lesson, not its empirical cut parameters or full optimizer. [Bonnabel et al.](https://arxiv.org/html/1410.7632) §II–III explicitly analyze rematching and noise assumptions behind ICP covariance; fixed-correspondence inverseH is not universally valid. GICP's synthetic planar metric and unknown shared scan/map errors do not acquire statistical meaning merely by inversion. [small_gicp source/paper](https://joss.theoj.org/papers/10.21105/joss.06948) supports reusing parallel preprocessing/registration; it does not promise a calibrated H-derived pose noise model.
