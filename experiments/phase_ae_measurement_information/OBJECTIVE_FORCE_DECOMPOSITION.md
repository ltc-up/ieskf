# Unchanged AD objective, multiplicity and force decomposition

For each Source i let k_j=(1-d_j²/R²)^3 inside original R, zero outside; rho_j=original Huber(r_j); tau=h²; b0=exp(-Huber(R)/tau). Define D=1+sum k, Z=b0+sum k exp(-rho/tau). E=0.5||L^-1 z||² -(tau/sigma²) sum_i log(Z_i/D_i). AD code and parameters are unmodified.

## Exact duplicate response

For m aliases of EVERY Target, write K=sum k, B=sum k exp(-rho/tau). Then

E_i(m)=-(tau/sigma²) log[(b0+mB)/(1+mK)],
dE_i/dm=-(tau/sigma²)(B-b0 K)/[(b0+mB)(1+mK)].

For valid unit normals and d<=R, |r|<=R, hence B>=b0 K and duplicate energy is nonincreasing. This is NOT m independent new observations: there is still one source contribution. It changes the plane-versus-null latent prior, generally moves gradients/Hessians and saturates as m→infinity. Local nonuniform sampling also changes relative plane probabilities. A denominator alone does not make the model density invariant. This analytic mechanism is confirmed at both actual frozen roots and on a known plane; no new optimizer was needed for the explanation.

## Complete force terms

Let v_j=grad_y rho_j=HuberPsi(r_j)n_j, u_j=grad_y k_j, a_j=k_j exp(-rho_j/tau)/Z, U=sum exp(-rho/tau)u/Z, V=sum a v, gD=sum u. Then

g_y E_i=(V -tau U +tau gD/D)/sigma².

These are respectively plane-residual, kernel numerator, and candidate-normalization/density force. Pull back by the exact map/IMU/base lever-arm and full15 fixed error chart, then whiten with SAME prior L. Bias/velocity have no direct geometry derivative; prior cross-covariance remains intact.

|Root|Prior white norm|Plane norm|Kernel numerator norm|Normalization norm|Norm of FULL vector sum|
|---|---:|---:|---:|---:|---:|
|G1|1.424878|1895.359509|1304.471633|702.323045|6.13e-10|
|G2|1.243006|377.591151|641.783479|295.009800|1.21e-09|

Norms cannot be added or interpreted as contribution percentages; the vector cancellation is checked directly. Full per-state vectors are private, with signed matrix/directional summaries public.

## Independent known-plane counterexample

Exact infinite plane z=0, noiseless Sources, known pose, original Huber/sigma/kernel and default SYNTHETIC initial prior (X/Y sigma1m, yaw5°). This synthetic prior is separate from the frozen real-input P; no real model was changed. All surface normals give ZERO intrinsic X/Y/yaw information.

Yet A's sampled-grid energy gives sigma X=Y≈0.074484m and yaw≈0.650488°. Duplicate2 yields X/Y≈0.098173m, yaw≈0.853975°; alternating-half gives X/Y1m and yaw5°. Thus repeated sampling need not monotonically increase confidence: the nonlinearity is not independent-information accumulation. Half-space duplicate density creates a tangential force and moves the optimum0.054425m from the known prior/true position while all plane equations still describe exactly the SAME infinite plane. Plane residual geometry cannot justify this tangential curvature. The artificial finite grid represents sampling, not a physical surface edge or landmark.


Evidence boundary: real-input values above are deidentified diagnostics, synthetic controls are explicitly separate, and historical AD results are reused rather than repeated. No absolute accuracy, statistical calibration or production qualification is inferred. Full Source/map/normal/state/P/trajectory records, factor identities and matrices remain in ignored local data. Proposed future model work is not an executed fix.
