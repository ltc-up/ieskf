# Selective / Schmidt / consider-state analysis

Use linearized residual r=Ha ea+Hs es+n, E[enᵀ]=0, covariance P partitioned into Paa,Pas,Pss; R=Cov(n). Both cross blocks matter:

S=Ha Paa Haᵀ+Ha Pas Hsᵀ+Hs Psa Haᵀ+Hs Pss Hsᵀ+R.
Ca=Paa Haᵀ+Pas Hsᵀ; Cs=Psa Haᵀ+Pss Hsᵀ.

Minimizing active mean-square error subject to no Schmidt mean update gives Ka=Ca S⁻¹, Ks=0. This constrained optimum is derived from minimizing tr(Paa−Ka Caᵀ−Ca Kaᵀ+Ka S Kaᵀ). The covariance is the error covariance of this estimator, not the fully conditioned Bayesian covariance:

Paa+=Paa−Ca S⁻¹ Caᵀ;
Pas+=Pas−Ka Csᵀ;
Pss+=Pss.

Equivalently for this constrained gain L: P+=(I−LH)P(I−LH)ᵀ+LRLᵀ. This preserves cross-correlation and PSD. Using P+=(I−LH)P alone is invalid for arbitrary L and can be asymmetric. Constructing L with protected zero rows implements the formal constraint only when the correct S/cross/Joseph model is retained; it is not permission to mask K in the old update. The implementation derives the active rows directly and never erases P blocks.

A consider-state is an uncertain nuisance retained in residual statistics but not estimated by that measurement. Selecting current velocity/bias as consider states is a deliberate information sacrifice, unlike the usual matured-map-feature use. Active updates can still be wrong. Subsequent propagation mixes states; partitions and reference changes require consistent Jacobians. ESKF injection needs SO(3) reset of every covariance block, including cross blocks. A protected attitude needs especially careful chart treatment. We do not implement a general nonlinear Schmidt15D backend here.

For valid measurement subspace U (rows orthonormal in a declared, unit-normalized tangent chart), use r'=Ur,H'=UH,R'=URUᵀ and update once. Projection discards unsupported observation components, not state correlations: velocities/bias can still change through PH'ᵀ. To protect those means combine projection with a justified Schmidt partition. Geometric eigenvectors require a metric/length scale because meters and radians cannot be ranked without normalization. This is a local linear statement; observable subspace varying with data and prior may add selection bias.

If noise and prior correlate, let N=E[e nᵀ]. Then C=PHᵀ+N and S=HPHᵀ+HN+NᵀHᵀ+R. Error covariance for gain L includes −(I−LH)NLᵀ−LNᵀ(I−LH)ᵀ. Independent-noise Joseph is insufficient. Same-scan multi-starts share noise; choose one grouped observation or retain a mixture, never multiply posteriors. Reusing the same residual as a new independent update is double counting.

[Synthetic results](SYNTHETIC_VALIDATION.md) verify analytic scalar and information/joint-error matrix references. Full trace1.31127 versus Schmidt1.80141 and projected2.36750 demonstrates information loss. Protected mean0 and variance1 are intentional, not a calibration achievement.

Primary reference: [Geneva, Maley, Huang, An Efficient Schmidt-EKF for 3D Visual-Inertial SLAM (2019)](https://arxiv.org/abs/1903.08636). It retains active–nuisance correlations while holding mature feature states. Our equations are independently derived above; its visual map partition is not automatically a suitable inertial partition.
