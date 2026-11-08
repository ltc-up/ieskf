# Failure inputs and selection
Selected before AP execution in TEST_PLAN: whole loop44 observation timing with its complete AO local trace, and the existing five predetermined frozen AM geometry/multistart groups. No after-the-fact favorable window, pose-aligned truth or tuned failure-frame seed was selected. Input SHA/Source identities are in the manifest and AO manifest.

Real evidence: whole-record local vertical failure, raw GNSS fix-code/continuity variation, historical competing CUDA endpoints and map support. Unverified labels: historical true wrong peak, surveyed map-hole location/extent, independent high-speed pose truth, runtime LiDAR subset handling and GNSS recovery accuracy. Do not convert absence of labels into zero errors.

Artificial conditions are separately declared:20m Target cylinder deletion; first NDT translation/duplicate/delay/two-hypothesis injections; middle-third GNSS/NDT absence; GNSS600ms delay and raw-LLA offset. The unchanged raw input is the control. Index/time choices precede results. Whole loop7/loop8 failures are retained from AO, not re-tuned in AP.

The first wrong-candidate shadow probe has no same-time odom bracket at startup and is rejected; it was not silently moved to another frame. Its reason was added to runner instrumentation and the first complete nine-scenario result retained as `fault_attempt1.json`. This is a failed probe prerequisite, not successful wrong-peak rejection. Synthetic unit tests separately show that a bracket-valid wrong shadow pose can be published untrusted while leaving local history unchanged.
