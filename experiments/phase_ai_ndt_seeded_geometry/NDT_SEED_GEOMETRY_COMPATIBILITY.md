# NDT compatibility with the point-plane MAP

NDT convergence is a property of its voxel objective; fitness is not the Huber point-plane MAP or a pose-truth error. Scores below separate factors frozen at the prediction from factors queried at each evaluated pose. Only a single column with identical factors defines a comparable fixed objective. Reselecting/adding/removing factors changes the objective and cannot establish global improvement.

| Event | Prediction MAP | NDT MAP, own factors | NDT MAP, prediction factors | NDT prior cost | NDT supported / unique Target | Complete seed guard |
|---|---:|---:|---:|---:|---|---|
| E1 | 1478.3672 | 255.8537 | 1042.3340 | 0.9989 | 896 / 782 | feasible |
| E2 | 573.2325 | 266.8884 | 359.2408 | 71.0640 | 857 / 750 | gyro_bias |
| E3 | 581.5372 | 208.8020 | 347.3411 | 54.7448 | 746 / 655 | gyro_bias, accel_bias |
| E4 | 1323.4877 | 3570.4129 | 4221.2694 | 3289.1016 | 1112 / 946 | accel_bias |
| E5 | 33.1571 | 159.2151 | 162.5756 | 124.6560 | 702 / 499 | feasible |

E4/E5 demonstrate that an NDT-converged seed can have higher point-plane MAP cost, especially its original-prior penalty. All seed point-plane local residual certificates fail before optimization. E1 has better initial support, but that alone does not establish an optimum. Own-factor scores at historical accepted states, residual/gradient statistics, margins and full comparison data are included in key_results.json. No NDT covariance or Hessian is read into the solve.

