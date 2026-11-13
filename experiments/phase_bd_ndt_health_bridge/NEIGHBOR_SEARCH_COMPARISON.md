# DIRECT7 versus actual research KD-tree

Existing C++ backend has DIRECT7/DIRECT27/RADIUS/WIDE modes, no KD-tree. RADIUS is a27-cell stencil with Gaussian-mean radius filtering; it is not renamed a tree. A separate research backend now builds a balanced3D KD-tree on the unchanged target Gaussian means, uploads nodes once and performs uncapped GPU radius traversal per transformed source. Target statistics, q50, source/target/seed/crop, derivative kernel and optimizer are shared. Active-region membership uses the same original voxel keys/voxel-center rule, not a new Gaussian-mean crop. No Rust backend or production source was replaced.

Radius=resolution1.5m. For means within their grid cells, any mean in a resolution-radius sphere lies within the27-cell stencil. This allows an independent RADIUS oracle for this radius only.40 real frozen one-step cases agree exactly in final pose, objective, gradient, Hessian and valid-pair count between KD and RADIUS; DIRECT7 agrees in pair count on0/40 and differs in gradient/Hessian. Independent public KD-builder CPU oracle passes84 planar/boundary/negative-coordinate/invalid-cell queries. KD mode was revalidated after using the public builder header. Larger-radius quality equivalence is not claimed.

The final native-quality calculation uses the equivalent radius stencil at this fixed radius; optimizer association uses the actual tree. PCL-like Fitness still uses the unchanged raw map. No neighbor cap truncates tree support. Full5872 common-lap matches and84 historical12632 perturbation matches are retained. Fixed step epsilon .001, relative condition disabled, max35 and step.2 follow P0; OFF and MT are both explicitly represented.

|neighbor|search|XY RMSE75 m|lateralP95 m|converged/1468|alignP95 ms|historicalHIGH/21|
|---|---|---:|---:|---:|---:|---:|
|direct7|OFF|0.624778|0.412082|1447|3.336|20|
|direct7|MT|0.590970|0.391564|1468|2.740|6|
|kdtree|OFF|0.616430|0.388446|1403|5.684|0|
|kdtree|MT|0.585107|0.381746|1468|4.209|0|

The historical21-start grid here is a NEW fixed grid on12632 (seven lateral offsets ×three yaw offsets), not the older462.5s21-grid. HIGH/LOW are nearest historical branch proxies, not certified truth. KD returns LOW for21/21 with either optimizer, whereas DIRECT7 OFF reaches HIGH20/21 (some are max-iteration endpoints). MT changes DIRECT7 basin coverage strongly but does not rescue KD. Objective scores cannot be ranked between different supports.

KD+MT modestly improves frozen overall RMSE but normal8 lateralP95 worsens from .4571 to .4856m; its historical basin coverage fails. KD+MT alignP95≈4.21ms vs DIRECT7 MT2.74ms. One-time full target tree build:60362 nodes,1207240 bytes, about7.8ms; historical local target7925 nodes/.16MB/about.94ms. These are tree costs, not full map-build latency. Traversal and Hessian are fused in linearization, so separate kernel costs are not fabricated. CPU maintenance/load can affect these instrumented research timings; only selected DIRECT7 has a separate actual full-cycle measurement.

Decision: NO-GO replacing DIRECT7. Mixed aggregate gain plus normal8 regression, wrong historical mode and higher latency do not meet the predeclared evidence gate for a new KD closed-loop candidate. No KD1468/4134 causal run is claimed. This is a deliberate negative experimental decision, not substitution of another backend's results. More map scenes would be required for a general ranking.
