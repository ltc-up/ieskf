# BF0 storage management

CONFIRMED (2026-11-15): new volume is NVMe NTFS mounted read-write; Samsung PSSD T7 is USB exFAT mounted read-write. Exact device/UUID/mount and machine paths are retained only in the private storage audit. Access checks succeed; no write benchmark or T7 probe file was created. A read-write mount/access bit is not a measured sustained-write or connection-stability guarantee.

| Disk | Total filesystem bytes | Used bytes at initial audit | Available bytes | Appropriate role |
|---|---:|---:|---:|---|
| New volume |1455121494016|1101625450496|353496043520|KITTI raw/derived/map/results/tmp; research code|
| T7 |1000169668608|833953595392|166216073216|Possible later RACECAR read-only source; no current writes|

Metadata/papers subsequently consume roughly tens of MB; recheck before download. Do not add the two free-space values together. exFAT lacks normal Unix hardlink/symlink/permission semantics and journaling; NTFS3 is also not a native Linux database filesystem. Keep SQLite bag read-only, avoid creating duplicate ROS bag databases, handle removable-disk loss explicitly. USB throughput/latency is unmeasured; if used later, separate read/decode/input wait from compute, with warm-cache and cold-cache results. In-memory input tests alone do not certify end-to-end timing.

## KITTI incremental peak budget on new volume

| Item | Reserved size |
|---|---:|
| Three archives, retained |814833599 bytes|
| Raw extraction, conservative all-member upper bound |814373473 bytes|
| Conversion / minimal index |0.5GiB|
| Frozen map |0.5GiB|
| Trajectories, full necessary R/K/P/states, diagnostics |1GiB|
| Temporary working set / logs / Python cache |1GiB|
| BF0 papers, manifests and allocation overhead |0.25GiB|
| Safety margin |remainder to8GiB|
| Incremental reservation |8GiB|

Selective raw extraction expected150605145bytes; full extraction is an upper budget, not a reason to extract images. Allocated sizes on disk differ from logical archive member sizes.8GiB is a proposed capacity budget, not a measured runtime footprint. No automatic archive deletion. No per-mode duplicate clouds; shared immutable map/Source/Target, small identity references instead. Disallow per-frame native/common point caches by default.

Stop new writes if remaining capacity below50GiB on new volume, if8GiB BF budget is reached, or if mount identity changes. Do not auto-clean historical data. TMPDIR, XDG_CACHE_HOME, PYTHONPYCACHEPREFIX and ROS_LOG_DIR must resolve to the work disk before an approved executable runs.

## Future RACECAR reservation, not approval

Candidate M db3=20711575552 bytes; metadata=16132bytes, uncompressedSQLite, no archive expansion. E db3=990986383360bytes does not fit either disk and is not selected. M estimated future reservation64GiB:19.3GiB source,8GiB bounded conversion/index,4GiB map,8GiB results,4GiB temporary, remainder margin. Both disks individually could fit M on today's audit; default recommendation remains new volume for simpler I/O, reconsider only after KITTI actual usage. No claim that this reservation already qualifies map/time/reference contracts. No M download before separate approval.

## Historical data and concurrency

Prior user-approved BE duplicate-cloud cleanup remains authoritative. No new historic recursive hashing, deletion, compression, moving, restoration or large du walk performed. Existing source, failed cases and alias mappings remain unchanged. Space is ample for KITTI; no cleanup candidates need approval now. Scoped name search at depth3 found no prior KITTI/RACECAR scene outside the newly created BF metadata; this is not an exhaustive hidden-file inventory.

Within visible /proc there were no open write descriptors to either work disk, and no actual replay/matching/download process identified. Sandbox process visibility is limited: not proof of exclusive use. Recheck before an approved run, never stop another process without authorization.
