# Reproduce AH

Public independent SE3 operator checks need Python3/NumPy/SciPy:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p test_deskew_ablation.py -v
```

The B test is kernel-base translation removal; full rotation/extrinsics still produce lever height. C uses Rz(deltaYaw)Rref. Original wrong IMU-origin B is excluded, not a selectable final mode.

Real-data replication requires authorized original raw bag, saved AA x/P and binding, same A2 motion cache, original private static transforms/preprocessor and CUDA-NDT binaries. These restricted inputs/source/SDK adapters are deliberately NOT distributed. Public math/tests independently reproduce; numeric real-data tables do not reproduce from a GitHub clone alone. Set A2RL_ROOT, IESKF_DATA_ROOT, AH_MOTION_FILE and AH_BAG locally. Keep the ROS environment in PYTHONPATH when invoking the extractor:

```bash
PYTHONPATH="src:$PYTHONPATH" python3 experiments/phase_ah_deskew_z_ablation/prepare_inputs.py --data-root "$IESKF_DATA_ROOT" --audit-root "$A2RL_ROOT/audits" --motion-file "$AH_MOTION_FILE" --bag "$AH_BAG" --output "$IESKF_DATA_ROOT/phase_ah_deskew_z_ablation"
```

Private adapter contract: original checked DeskewCudaProcessor code, SAME double/float kernel for all modes; AH_MODE0=A,1=B(world delta pBaseZ=0),2=C; original adapter separately validates A. The local construction scripts and exact binary/source hashes are preserved locally. Run each mode on jobs.json into A/B/C folders with original preprocessing/native rebase. Do not replay legacy preintegrator or generate a new motion trajectory. Current final B does not consume an IMU lever override.

```bash
PYTHONPATH="src:$PYTHONPATH" OPENBLAS_NUM_THREADS=1 python3 experiments/phase_ah_deskew_z_ablation/run_frozen.py --data-root "$IESKF_DATA_ROOT" --output "$IESKF_DATA_ROOT/phase_ah_deskew_z_ablation" --matcher "$AH_UNCHANGED_MATCHER"
PYTHONPATH="src:$PYTHONPATH" OPENBLAS_NUM_THREADS=1 python3 experiments/phase_ah_deskew_z_ablation/summarize_saved.py --data-root "$IESKF_DATA_ROOT" --output "$IESKF_DATA_ROOT/phase_ah_deskew_z_ablation" --public-output /tmp/ah_key_results.json
```

The private constrained diagnostic protocol/data is intentionally reported as failed, not a missing successful result. Existing unchanged score service and constrained_ndt.py document attempted method; raw original SDK/adapter and all failed results stay local. Corrected B diagnostic is NOT_EXECUTED. Do not interpret optimizer-success as converged MAP, or repeated Source updates as new observations. Full filter trajectories/poses/raw moments are excluded from Git; public summary is a whitelist. No ROS publisher or continuous replay is used. Plot reproduction needs only public key_results.json.

```bash
python3 experiments/phase_ah_deskew_z_ablation/plot_results.py --results experiments/phase_ah_deskew_z_ablation/key_results.json --output /tmp/ah_sensitivity.png
```
