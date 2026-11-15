# BF0 dataset feasibility and KITTI download request

Audit date 2026-11-15; all sizes bytes unless specified. Approval state PENDING. Dataset payloads must not be fetched automatically by a reproduction command.

## Fixed selection

CONFIRMED: official KITTI [raw page](https://www.cvlibs.net/datasets/kitti/raw_data.php) describes raw and synchronized streams; current web download UI requires login. Direct official S3 objects respond to HEAD. Their public availability does not replace the user's approval or dataset terms.

Only 2011_09_26_drive_0002 is requested. ZIP central-directory metadata, without extracting members, identifies 77 synced LiDAR .bin files/77 synced OXTS records and 846 extract OXTS records. The extract LiDAR members are .txt, not the sync .bin representation. No second scene, camera labels or tracklets are requested.

| Official archive | Download bytes (HEAD) | All-member uncompressed bytes (ZIP directory) |
|---|---:|---:|
| [2011_09_26_drive_0002_sync.zip](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip) | 319776816 | 319651180 |
| [2011_09_26_drive_0002_extract.zip](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_extract.zip) | 495052715 | 494718821 |
| [2011_09_26_calib.zip](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip) | 4068 | 3472 |
| Total | 814833599 | 814373473 |

Three archives total 814.834 MB / 0.759 GiB. Keep the archives after approved download; no automatic deletion. Selectively extract sync velodyne_points/oxts, extract oxts, and the three calibration text files. These selected directories total 150605145 logical bytes by ZIP metadata (including directory/timestamp entries); no camera extraction, no duplicate raw LiDAR conversion. Archives contain images that are unnecessary but cannot be excluded from an ordinary full ZIP transfer. A range-based payload downloader is not proposed.

HYPOTHESIS: extract's 846 timestamped OXTS samples supply approximately100 Hz inertial channels over the short recording. Timestamp intervals, interpolated-invalid flags, specific-force semantics and body axes remain to be verified after approval. The sync OXTS series alone is not high-rate IMU. Calibration is present by filename; values have not been extracted or verified.

## Storage proposal — KITTI only

Use new-volume local storage (scheme A). Work root is defined in the private path audit; subdirectories kitti/downloads, kitti/raw, kitti/derived, maps/kitti, results/kitti, tmp and logs are all on that same volume. Current free space about353.45GB /329.18GiB; reserve8GiB incremental peak including safety margin. Expected free after full reserved use remains about321GiB. T7 is not needed for KITTI. No historical cleanup requested or authorized.

[Peak itemization](STORAGE_MANAGEMENT.md) deliberately budgets full extraction even though selective extraction is planned. If archive size, scene, work disk or required peak exceeds this plan materially, stop and request revised approval. If the device disappears, stop; do not fall back to home/tmp. Official login/access failure also stops acquisition; do not bypass access controls.

## Experiment scope after approval

1. Download only the three exact URLs above; retain HTTP identities, archive hashes and ZIP integrity validation. No matching until input/frame/time contracts pass.
2. Sensor adapter audit: raw body angular rate/acceleration only; OXTS pose/velocity remains reference or one declared initialization, never continuous prediction feedback. Wheel/Kistler absent: both B0/B1 disable them. A is Shadow only; C prohibited.
3. Predeclare map scans0–19, test scans20–76 (sync indexing), excluding every test point from map. This is a proposed functional holdout, not certified coverage. Use only map-side OXTS poses for this explicitly reference-assisted map and at most one declared initialization; test OXTS poses cannot become seeds. No posthoc split changes for results. If static overlap is inadequate, report blocked map localization rather than accumulate test clouds or acquire another scene.
4. Freeze Source/Target/seed/final match/native parameters, prior/P, observation identity/time and full R. Compare B0 Legacy and B1 Native Raw with common necessary front-end validity checks; record any unchanged Legacy-specific admissibility separately. No forced rejection of valid Native solely by Fitness. No Native×Fitness/diagonalization/calibration or matcher retuning.
5. Only after input and map feasibility, run the same tiny scene causally with its own state and real delays. GT cannot select modes/parameters/R. Detailed outputs are in the covariance plan. This authorization request does not cover additional scenes or new algorithms.

## What this cannot establish

UNKNOWN: independent full-track point map, high-quality independent6D truth, absolute Native scale/NEES, long-term bias convergence, racing/banking generality and worst-case real-time guarantees. ~77 scans is a functional demonstration, not independent generalization accuracy certification. KITTI nominal10 Hz does not relax the study's50 ms cycle target. No preapproved fallback scene.

请批准KITTI场景下载及对应存储方案。
