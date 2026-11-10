# Numerical validation and separate failed model controls

Final regression **205 passed,0 failed,0 skipped**:183 frozen baseline +12 unchanged AW +10 AX. Both AW_GICP_BINARY and AX_BINARY were exported, so actual C++ executable tests ran. One earlier targeted command omitted AX_BINARY and skipped3 tests; its log is retained and is not the final qualification.

New tests: complete15D lever/right-SE3 finite differences; joint21D nuisance/state finite differences; single-prior linear analytic update with velocity cross-covariance; atomic rejection; dense versus shared latent solve; actual C++ accumulated H versus dense correlated covariance; rank3/5/6 single/two/three-plane geometry; exact Source/Target duplicates; frozen-factor cost-gradient finite differences; shared information budget; delayed/on-time direct-update replay equality and duplicate rejection. Tests are grouped into10 unittest methods.

Actual off-manifold frozen real-Source6-axis gradient finite difference max discrepancy5.76e−6, with~100-scale rotation gradients. This validates the linearized-common service objective, not its successful nonlinear filter behavior (that variant fails). Final exact nonlinear nuisance chart is independently finite-differenced; actual full-lap reconstructed joint marginal covariance agrees with saved Q15 to5.26e−13. Joseph identities and reset preserve full cross terms.

Fresh original-mode40 runs real CUDA-NDT/Deskew and reproduces original AR seed,full priorP,final pose,K,15D delta exactly (all max0). A instrumented1468 repeat has identical acceptance, seed,priorP and complete delta (max0); only diagnostic logging/timing changed. Added diagnostics expose complete native prior state, iterate charts and RPC timings without changing the algorithm.

Expected scientific failures are separate from unit failures: full GICP/NDT3D metric creates plane tangential information; native CUDA kernel map duplication changes shape; refresh-every-step cannot converge under the budget; patch-only and linear-common real candidates fail. Positive-definite matrices do not erase these failures.
