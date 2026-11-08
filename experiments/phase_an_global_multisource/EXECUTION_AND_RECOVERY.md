# Execution and recovery evidence

Public118-unit-test run passes (10 new AN,6 AM,8 AL,94 earlier). run_recovery.py independently executes a SYNTHETIC fixture:5 position anchors,ambiguity pruned only by explicit independent joint bounds,RECOVERING→PARTIAL,COASTING,UNAVAILABLE and recovery. Unobserved yaw remains wrong.4rad. GLOBAL_TRACKING is tested only with a separately qualified initial-orientation fixture; this does not make GNSS a yaw sensor. No synthetic result is a real GNSS recovery claim.

Real event manager uses20ms emulated processing/buffering delay to ensure legal past-time local interpolation. This is a deterministic offline scheduler,not observed callback latency or a production closed loop. Poses are added in chronological arrival order; delayed measurements use their own Header time,no stale latest-odom subtraction. Decision reasons and statuses are retained privately per event; public reports expose aggregates only.

AN ROS2 node actually started on isolated domain174,/research/fusion/global_input,global_status,global_correction. It transported3654 real-input-derived events and reproduced offline states/reasons:1470 status slots,0 global correction publications,no TF. Nodes use schema1 std_msgs/String envelopes; independently built typed message contracts from AL remain available but native typed production adaptation is not implemented. This validates research transport/abstention,not live sensor synchronization or safe production control.

Repeated saved-input manager runs were used for source-health/transport-field completion; no new local propagation,matching,GNSS selection or parameter tuning was done in AN. GNSS bag was read once for extraction and its selected CDR-group digest is fixed. Full records are ignored,not public.
