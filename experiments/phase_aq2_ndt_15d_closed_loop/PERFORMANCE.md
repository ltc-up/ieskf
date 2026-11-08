# Measured performance
Per-branch P50/P95/P99/max wall times are in A–E JSON; these include the actual processing transaction rather than summed module percentiles. C40 typical wall~13ms, max<20ms on this run; C20 evaluated prefix P50~12.66ms,max~15.48ms. Persistent GPU/map startup is approximately0.26–0.30s and reported separately. These figures include this independent wrapper/data path, not production ROS scheduling.

The original per-frame executable first request took~459ms plus sensor delay, exceeded500ms history, and failed. That failure is not hidden by warm steady-state timing. Private persistent service creation preserves CUDA algorithm and preprocessing but changes resource lifetime. Its source/library/binary identities are in the manifest; restricted adapter source remains private.

No end-to-end20Hz or100Hz ROS2 qualification is issued. Offline input availability, dropped/aborted frames, hardware-clock uncertainty, source preprocessing, acquisition phase and scheduling differ from a live stack. Even adequate compute does not cure the C20 observation-retention failure.
