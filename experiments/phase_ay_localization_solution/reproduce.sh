#!/usr/bin/env bash
set -euo pipefail
: "${AY_PRIVATE:?Set a new private persistent output directory}"
: "${AY_BAG:?Set the authorized loop44 SQLite file}"
export PYTHONPATH=src OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
export IESKF_CONFIG=data/history/phase_v/config/frozen.json
mkdir -p "$AY_PRIVATE"
# Source the installed ROS environment and authorized message definitions first.
python3 experiments/phase_ay_localization_solution/extract_optical.py --bag "$AY_BAG" --bindings data/phase_as_accuracy_quality/bindings_multilap.json --output "$AY_PRIVATE/optical.json" --audit "$AY_PRIVATE/optical_audit.json"
for specification in baseline40:baseline:lap:40 L_lap:optical_lateral:lap:1468 L_multilap:optical_lateral:multilap:4134 XY_lap:optical_xy:lap:1468 XY_multilap:optical_xy:multilap:4134 C_lap:optical_xy:lap:1468 C_multilap:optical_xy:multilap:4134; do
 IFS=: read -r name policy window count <<< "$specification"
 bindings=data/phase_ar_loop44_map0105/bindingslap.json
 [[ "$window" == multilap ]] && bindings=data/phase_as_accuracy_quality/bindings_multilap.json
 extra=(); [[ "$name" == C_* ]] && extra+=(--calibrate-yaw)
 python3 experiments/phase_ay_localization_solution/run_loop.py --input data/phase_ao_ap_validation/loop44.json --bindings "$bindings" --target data/phase_ar_loop44_map0105/map0105_target.bin --deskew data/phase_ar_loop44_map0105/runtime/deskew_service --deskew-service data/phase_ar_loop44_map0105/runtime/deskew_service --matcher data/phase_ar_loop44_map0105/runtime/match_fixed100 --optical "$AY_PRIVATE/optical.json" --private "$AY_PRIVATE/$name" --output "$AY_PRIVATE/${name}_summary.json" --policy "$policy" --window "$window" --limit "$count" "${extra[@]}" > "$AY_PRIVATE/$name.log" 2>&1
 [[ "$name" == baseline40 ]] && continue
 baseline=data/phase_ar_loop44_map0105/fullpose100_lap;asref=data/phase_as_accuracy_quality/single_lap/evaluation.npz
 [[ "$window" == multilap ]] && baseline=data/phase_as_accuracy_quality/F_multilap && asref=data/phase_as_accuracy_quality/multilap/evaluation.npz
 python3 experiments/phase_ay_localization_solution/evaluate.py --matching-backend ndt --run "$AY_PRIVATE/$name" --baseline "$baseline" --motion data/phase_ao_ap_validation/loop44.json --as-evaluation "$asref" --at-bindings data/phase_at_gnss_footprint_corners/bindings.json --reference-cache data/phase_aw_native_gicp_fusion/private/gicp_multilap/evaluation/evaluation.npz --run-summary "$AY_PRIVATE/${name}_summary.json" --private "$AY_PRIVATE/$name/evaluation" --output "$AY_PRIVATE/${name}_metrics.json" --label "$name" --expected "$count"
done
# Profile separately with --profile, a fresh --private directory and unchanged policy.
