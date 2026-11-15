# Phase BF / BF0 — download approval checkpoint

Status: AWAITING_KITTI_APPROVAL. No BF localization experiment has run. Baseline da3ea9df0acf34eb5c0808c2e258c9bbb86bafb1, existing research branch; main is unchanged.

The one KITTI scene is 2011_09_26_drive_0002. The one future RACECAR candidate is S3/M-SOLO-FAST-100-140. This checkpoint requests KITTI only, not RACECAR or historical cleanup.

[Feasibility and request](DATASET_FEASIBILITY.md), [storage](STORAGE_MANAGEMENT.md), [literature](RACECAR_LITERATURE_REVIEW.md), [KITTI input/map contract](KITTI_INPUT_AND_MAP_AUDIT.md), [RACECAR metadata audit](RACECAR_INPUT_AND_MAP_AUDIT.md), [Git/audit limitations](STORAGE_AND_GIT_AUDIT.md), [reproduction](REPRODUCE.md).

CONFIRMED means a source/metadata check, not successful adaptation. HYPOTHESIS means a proposed experiment. UNKNOWN means not established. Results files are deliberately NOT_RUN. Numerical BE5 tests are not dataset accuracy validation.

Literature limitation: the original SiPS 2024 full PDF is inaccessible (HTTP403); its abstract/record is reviewed, and the author's earlier thesis summary was separately read. These are not interchangeable versions. The remaining SiPS session/latency/full evaluation claims stay UNKNOWN. This limitation does not require downloading either dataset.
