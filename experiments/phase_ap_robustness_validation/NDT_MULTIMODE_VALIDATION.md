# Multimode / wrong-peak evidence
Re-applied the frozen complete-link clustering to the stored five seven-start CUDA groups, preserving every fitness and convergence result. Two groups remain AMBIGUOUS, three UNKNOWN; no hypothesis is counted as independent LiDAR information. These are historical AM GPU outputs, not35 new AP fits or a full-lap peak-discovery rate.

No independent labels certify which endpoint is the true map pose. A low fitness, convergence label or rank6 cannot resolve this. A score difference is not a calibrated likelihood ratio; neither a real wrong-peak precision/recall nor a false-alarm rate is reported.

The whole-record first-NDT explicit two-hypothesis injection supplies two poses10m apart to the unchanged manager. It produces one AMBIGUOUS_NO_ANCHOR decision and40 of91707 output slots AMBIGUOUS before freshness expires; it never becomes a trusted anchor. GNSS is unqualified, so it cannot prune the two modes. When evidence expires the system is UNAVAILABLE, not normal tracking. This tests preservation of supplied ambiguity, not automatic discovery of a hidden real peak.

The first +100m X/+50m Z candidate remains UNKNOWN at the real manager and creates no anchor. The shadow probe at that fixed startup frame lacks an odom bracket, so it does not test real wrong-anchor propagation; this failure is retained. A synthetic regression separately demonstrates that wrong bracket-valid shadow correction may publish an incorrect untrusted global pose without local mutation. Neither result proves real wrong-peak detection.
