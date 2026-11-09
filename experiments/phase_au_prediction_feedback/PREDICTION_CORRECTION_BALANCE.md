# AU2 — Exact continuous prediction/correction balance

**VERIFIED algebra and state increments; NDT/GNSS-referenced interpretation CONDITIONAL.** A moving vehicle's displacement is not called propagation error.

For current map-axis n_i, reference z_i (first NDT, then qualified engineering GNSS):

```
e_prior_i = n_iᵀ(p_prior_i − z_i)
inherited_i = n_iᵀ(p_post_(i−1) − z_(i−1))
new_difference_i = n_iᵀ[(p_prior_i−p_post_(i−1)) − (z_i−z_(i−1))]
update_i = n_iᵀ(p_post_i−p_prior_i)
e_prior_i = inherited_i + new_difference_i
e_post_i = e_prior_i + update_i
Δe_post_i = new_difference_i + update_i + basis_change_i
basis_change_i = (n_i−n_(i−1))ᵀ(p_post_(i−1)−z_(i−1))
```

All points are the same rear-axle reference and native Header time. The axis transport term is required in curves; summing scalar lateral changes while omitting it would invent accumulation. Maximum per-step balance discrepancy9.72e−17m; cumulative local closure likewise at roundoff. Nominal displacement decomposition (held-v dt, rotated specific force, accel-bias/gravity acceleration, rotating lever, direct wheel correction) closes to5.52e−13m. Its components are actual motion terms, not independently labelled errors.

| Section | Frames | Sum motion-minus-NDT displacement m | Sum NDT correction m | Sum axis transport m | Opposing motion/correction signs | Prior discrepancy grows from previous posterior |
|---|---:|---:|---:|---:|---:|---:|
|1|48|−.298|−.063|+.550|30/48|22/48|
|3|89|+1.047|−1.376|+.122|64/89|56/89|
|9|29|+.0168|−.2768|+.1531|14/29|13/29|

These sums include the incoming interval to the first section sample and must not be read as net physical drift. In3 there are frequent intervals where motion relative to successive NDT positions opposes the preceding correction. Nevertheless the section's posterior-NDT difference decreases overall. In9 the net new relative displacement is small; inherited discrepancy and axis transport dominate, and the posterior-NDT difference also decreases. There is no evidence of the same universal correction-cancellation mechanism in both right sections.

With engineering GNSS and the frozen AS normal, the valid consecutive-sample mean new difference/update is−.00373/−.01587m in3 and−.01235/−.01220m in9. Both mean signs reduce the positive difference in those coordinates. This different result is retained, rather than choosing the reference that best supports a desired causal claim. GNSS gaps are excluded, and GNSS's uncalibrated time/installation/attitude remain limitations.

Full canonical traces separate one application of every information event from its multiple delay-replay executions. There are7357 unique wheel corrections, not18609 independent wheel measurements. Real-time executor freshness and actual physical sensor delay are not inferred from this closed numerical balance.
