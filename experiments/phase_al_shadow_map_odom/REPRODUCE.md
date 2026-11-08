# Reproduce AL

Public checks from repository root:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p test_shadow_map_odom.py -v
```

Authorized private inputs require the exact V event/contract/config/Source files, complete Target and AJ transaction ledger. No payload is distributed. Set IESKF_CONFIG to the original frozen.json and preserve the contract digest. Run:

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src python3 experiments/phase_al_shadow_map_odom/run_real.py --data-root "$IESKF_DATA_ROOT" --private-output data/phase_al_shadow_map_odom --public-output experiments/phase_al_shadow_map_odom
source /opt/ros/jazzy/setup.bash
ROS_LOG_DIR=/tmp/phase_al_ros_logs ROS_DOMAIN_ID=174 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONPATH=src:$PYTHONPATH python3 -m ndt_global.ros_shadow --events data/phase_al_shadow_map_odom/ros_events.jsonl --report experiments/phase_al_shadow_map_odom/ros_results.json
colcon --log-base /tmp/phase_al_colcon_logs build --base-paths ros2 --build-base /tmp/phase_al_build --install-base /tmp/phase_al_install
```

DDS may need normal local network access. No production topics/TF. Raw decisions, pose/state/P histories and wire events stay ignored. Hash identities in results/manifest bind data without publishing trajectories. Research-code commit is the enclosing AL commit (`git log -1 -- src/ndt_global/shadow.py`).
