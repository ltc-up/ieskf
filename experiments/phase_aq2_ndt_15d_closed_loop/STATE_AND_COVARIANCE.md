# State and covariance behavior
Full15D mean/P are saved privately for every processed frame and every delayed observation transaction. Position, velocity, attitude and bias correlations remain intact; single Joseph/reset updates use the selected measured components. Numerical SPD is checked, not called consistency.

C40: max accepted NDT dZ~1.21m, max velocity increment~0.69m/s, final bg norm~0.027rad/s, ba~0.107m/s², model sigmaZ~0.208m. C20 stopped prefix: max pose dZ1.166m, max velocity delta1.139m/s, final bg0.0221rad/s and ba0.378m/s², sigmaZ0.441m. These are actual feedback effects; no independently calibrated bounds certify physical plausibility. Per-correction angle/bias gates do not imply an absolute calibrated bias bound.

No600m-style gain row is suppressed. Max actual wheel dZ in C40 is0.0216m vs A0.1048m and B0.1866m. Changing these values does not establish truth; full state differs because map observations change correlations and later matching. Native-reference A/E full-P equality is exact; their wall-time/current-stamp outputs are not directly compared as simultaneous states.

Historical fitness rule supplies exploratory R: translation and angular diagonals use the same documented legacy publisher/EKF scaling and smoothing multiplier. New Z uses the existing historical Z scaling, not a fabricated “calibrated” variance. Matching correctness, source reuse/seed correlation and statistical covariance are unresolved; no NEES/NIS PASS. AQ3 must separately assess candidate coordinates, observability, scaling and multimodal risk.
