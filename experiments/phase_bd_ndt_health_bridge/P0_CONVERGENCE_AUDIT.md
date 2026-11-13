# BD-P0 convergence source audit

CUDA `ndt_gpu_matcher.cpp`: raw Newton solve by SVD, normalized 6-vector direction, initial step clamped to [transformation_epsilon/2, step_size]. The vector mixes metres and Euler radians; this is a chart norm, not a translation-only tolerance. OFF still retains this clamp and direction handling.

With relative_objective_epsilon>0, ObjectiveConvergenceTracker requires two consecutive steps with norm<=transformation_epsilon AND abs(score_new-score_old)/max(abs(score_old),1e-12)<=relative_objective_epsilon. With it disabled, the legacy branch stops on accepted step<transformation_epsilon after more than one iteration. Thus these are alternative policies, not three independent OR gates. Exact zero raw Newton norm can exit immediately; rank/finite errors and MAX_ITERATIONS have separate reasons. No separate positive gradient-norm threshold was found in the reviewed Newton branch.

OMP defaults `params_.use_line_search=false`; its step helper only enters More-Thuente when enabled. The caller invokes the helper with lower step=trans_epsilon/2, and stops on step<trans_epsilon or nr_iterations>max_iterations. It marks max exhaustion converged and increments its counter afterward. Configured35 may report37; CUDA's strict iteration loop and explicit MAX_ITERATIONS differ. Same parameter name is comparable as a chart-step threshold, but not equivalent complete convergence semantics.

Current ROS YAML: resolution1.5, max35, step0.2, epsilon0.01, relative0.001, GLL3; C++ hardwires DIRECT7/q50 and enables search. Adapter propagates both epsilons. Global-init adapter independently hardcodes relative0.001 and More-Thuente. AY matcher100: DIRECT7/GLL3/epsilon0.01/relative0.001/max100. These are source-resolved values, not proof of a running node's loaded configuration.

Required controlled grid: DIRECT7; transformation epsilon {0.001,0.01}; OFF/MT; independently objective epsilon {0,0.0001,0.001}; max35 and a separately labeled100 iteration diagnostic if needed. Keep common source/target/seed/ROI/q50. Because transformation epsilon changes the lower step, add a fixed-min-step diagnostic before attributing changes exclusively to termination. GLL3 remains the frozen AY reference, never silently renamed OFF.

Final pose/reference error and closed-loop error remain unmeasured for BD. Numerical convergence, correct local branch, and vehicle accuracy remain separate outcomes.
