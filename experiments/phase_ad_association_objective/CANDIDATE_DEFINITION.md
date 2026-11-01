# One candidate frozen after AD1/AD2 direct evidence

## Evidence before choice
The original cycle segment has a finite selected-objective jump about 0.193896 as one-sided parameter offsets shrink from 1e-3 to 1e-8. The nearest-valid cycle also has finite jumps at switches with unchanged Source support. Fixed active-cell derivatives match finite differences. Thus merely changing the inner optimizer cannot give a coherent smooth global merit.

## Compact-support Huber plane mixture (ONE new objective)
For EVERY source i and every statically normal-valid map Target j with distance d_ij<R:

s_ij=d_ij^2/R^2; k_ij=(1-s_ij)^3, otherwise k=0.

Use the existing radius R, Huber delta h, sigma, immutable map plane-fit predicates. tau=h^2, c=Huber(R), b0=exp(-c/tau). Define

D_i=1+sum_j k_ij,
Z_i=b0+sum_j k_ij exp(-Huber(r_ij)/tau),
e_i=-tau log(Z_i/D_i),
E(z)=0.5 ||L^-1 z||^2 + sum_i e_i/sigma^2.

No nearest selection or changing observation population. An unsupported source contributes the constant c/sigma^2, not zero. All valid planes inside the SAME radius contribute; invalid planes have identically zero weight. k and its first two derivatives vanish at radius. This objective is C1 (Huber is C1), piecewise C2; Huber kink can change Hessian but not gradient. Null weight=1, tau=h^2 and polynomial exponent=3 are declared algebraic choices, not fitted outcome parameters; no alternate kernel/temperature is tried.

Gradient:

partial_y e_i=sum_j [(k_ij exp(-rho/tau)/Z_i) psi(r_ij) n_j
 + tau(1/D_i-exp(-rho/tau)/Z_i) partial_y k_ij],
partial_y k_ij=-6(1-s_ij)^2 (y_i-q_j)/R^2.

The second term is ESSENTIAL. Freeze only the map and prior, not candidate weights during differentiation. Exact radius query is a computational truncation of zero contributions; adding a plane at the boundary is C2. Nearest-plane ordering has no role.

This replaces the entire discrete association objective, INCLUDING discontinuous target-multiplicity normalization. Sigma/Huber are unchanged, but the effective measurement model/information IS CHANGED; old/new raw cost numbers cannot be ranked. There is no claim this is a minimal parameter tweak, equivalent likelihood, density-invariant model or validated posterior. Duplicated Targets/Source can still change information: test/report this limitation, never disguise it as calibrated R. Soft weighting is an explicit mathematical association candidate, not a new map or matcher backend.

## Solver and qualification
Reuse AC full15 chart, Gaussian prior and exact original cumulative guards/trust-step balls. At each bounded trust stage SLSQP minimizes this SAME dynamic objective using analytic full derivatives; original line scales assess the SAME objective. 60 stages / 200 inner iterations, ftol=1e-12. Same 13 starts. Stop only with prior-white raw gradient inf<=1e-4 for interior (never clipped-step size alone), or existing guard-KKT diagnostic at an active boundary. Verify final finite differences; no raw hard-pair identity certificate is required for smooth mixtures, but exact candidates, weighted support and objective/gradient repeatability are recorded. Support minimum remains an engineering check, not an objective discontinuity. No returned state/P is a posterior until separately validated. No hard-boundary diagnostic Gaussian posterior.
