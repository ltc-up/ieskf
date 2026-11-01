# Experiment Index

Historical phase records will be migrated from the local audit workspace after a publication review. The entries below summarize previously reported outcomes but are not accompanied by all raw evidence in this public repository yet.

| Phase | Focus | Current evidence | Status |
|---|---|---|---|
| U | 6DoF geometric factors | Synthetic numerical checks | Prototype |
| V | 15D geometry-ESKF offline replay | 14.7-second conditional replay | Not validated for accuracy |
| W | Deskew input contract and diagnostic frames | Original Bosch IMU, timing, and association audits | Conditional |
| X | MAP / line search / association | Fixed-association numerical consistency | Conditional |
| Y | 15D constrained MAP and 6+9D elimination | Five fixed cases KKT; dynamic association not converged | Conditional |
| Z | Z-velocity covariance attribution and posterior numerical design | Large candidate vertical velocity corrections explained; real updates rejected | Not production-qualified |
| AA (planned) | Deskew provenance and accepted Z-velocity spike verification | No new results yet | Planned |

For original reports and exact per-phase input hashes, use the local historical audit manifest until migration is completed. Do not infer statistical consistency or localization accuracy from numerical passing checks.
