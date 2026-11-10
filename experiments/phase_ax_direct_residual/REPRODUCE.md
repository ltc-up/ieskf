# Reproduce AX without publishing private payloads

Checkout research/phase-ax-direct-residual. Base c647ac3; AW code restored byte-for-byte from e15e7a2. Original authorized data directory is linked as ignored`data`; set AX_PRIVATE to a NEW persistent user-authorized output root (never/tmp). Existing AR/AS/AT/AW files, raw clouds, map Target, original CUDA Deskew/NDT binaries, MIT small_gicp headers and reference cache are separately required. Do not apply/drop any stash, modify production or overwrite run directories.

```
export PYTHONPATH=src
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1
export IESKF_CONFIG=data/history/phase_v/config/frozen.json
# Set SMALL_GICP_INCLUDE to authorized small_gicp upstream/include.
# Set AX_PRIVATE to private persistent output root.
export AX_BINARY="$AX_PRIVATE/runtime/direct_service"
sh experiments/phase_ax_direct_residual/build.sh
export AW_GICP_BINARY=data/phase_aw_native_gicp_fusion/private/runtime/gicp_service
python3 -m unittest discover -s tests -v
```

Actual A:

```
python3 experiments/phase_ax_direct_residual/run_loop.py \
 --input data/phase_ao_ap_validation/loop44.json \
 --bindings data/phase_ar_loop44_map0105/bindingslap.json \
 --target data/phase_ar_loop44_map0105/map0105_target.bin \
 --deskew data/phase_ar_loop44_map0105/runtime/deskew_service \
 --deskew-service data/phase_ar_loop44_map0105/runtime/deskew_service \
 --matcher "$AX_BINARY" --shared-frame \
 --private "$AX_PRIVATE/NEW_A_lap" --output "$AX_PRIVATE/NEW_A_lap_summary.json"
```

Use limit40 first. For actual continuous4134 use AS`bindings_multilap.json`, window`multilap`, fresh output names; no per-lap reset. Original500ms history and measured runtime stay intact. Mode without shared-frame is the retained failed patch-only model, not the final candidate. The service's separate linear shared-frame flag is a failed diagnostic; final observer always disables it and optimizes the finite nuisance in Python.

Native Gaussian exporter requires the original external CUDA library/include/internal CUDA headers, Eigen and cudart. Build export_gaussians.cpp linking the same authorized`libndt_gpu_matcher` used by AR; no source patch is required. Typical flags:

```
c++ -O2 -std=c++17 -I/usr/include/eigen3 -I"$CUDA_INCLUDE" \
 -I"$NDT_SOURCE/include" -I"$NDT_SOURCE/cuda" \
 experiments/phase_ax_direct_residual/export_gaussians.cpp \
 -L"$NDT_LIBRARY_DIR" -lndt_gpu_matcher -L"$CUDA_LIBRARY_DIR" -lcudart \
 -Wl,-rpath,"$NDT_LIBRARY_DIR" -o "$AX_PRIVATE/runtime/export_gaussians"
"$AX_PRIVATE/runtime/export_gaussians" data/phase_ar_loop44_map0105/map0105_target.bin "$AX_PRIVATE/gaussians.bin"
python3 experiments/phase_ax_direct_residual/raw_moment_validation.py \
 --target data/phase_ar_loop44_map0105/map0105_target.bin \
 --gaussians "$AX_PRIVATE/gaussians.bin" --output "$AX_PRIVATE/raw_moment_validation.json"
```

For B add`--gaussians "$AX_PRIVATE/gaussians.bin" --unregularized-planes` to the A command and use new B output names. Omit unregularized-planes only to reproduce the failed kernel-shape version. Original frozen noise, geometry eligibility and loss gate stay unchanged. B full-lap actually fails; do not report4134 executed. Synthetic controls: ndt_numeric.py with binary/output; ndt_density.py with exporter/private/output. The latter really executes the CUDA builder on synthetic exact copies.

Native-time evaluation:

```
python3 experiments/phase_ax_direct_residual/evaluate.py \
 --run "$AX_PRIVATE/NEW_A_lap" --run-summary "$AX_PRIVATE/NEW_A_lap_summary.json" \
 --baseline data/phase_ar_loop44_map0105/fullpose100_lap \
 --motion data/phase_ao_ap_validation/loop44.json \
 --as-evaluation data/phase_as_accuracy_quality/single_lap/evaluation.npz \
 --at-bindings data/phase_at_gnss_footprint_corners/bindings.json \
 --reference-cache data/phase_aw_native_gicp_fusion/private/gicp_multilap/evaluation/evaluation.npz \
 --private "$AX_PRIVATE/NEW_A_lap/evaluation" \
 --output "$AX_PRIVATE/NEW_A_lap_metrics.json" --expected 1468
```

For3laps use actual continuous run, ASF_multilap baseline, ASmultilap evaluation, expected4134. Reference cache exact Headers/masks are verified; missing original historical GNSSCSV is not reconstructed by fitting. Cached AT physical conversion remains CONDITIONAL. Original fully recorded raw/Deskew history allows reproduction with separately authorized assets; public aggregates alone cannot reproduce real sensor measurements.

Original rollback/reproduction uses unchanged AW run_loop.py`--backend ndt_legacy` with ARmatch_fixed100; AX fresh40 equals frozen AR exactly. diagnostics.py documents/private instruments native state/P/iterate charts; an actual instrumented lap gives exact numerical parity. `build_reports.py` alone regenerates deidentified aggregate figures from committed JSON; with private-root it seals new aggregate evaluations. Full trajectory overlays, corner plots, source data, state/P and all failed logs remain private.
