# B Source / Target contract

Current Source is exactly the common A0 preprocessed Source values: original three-LiDAR fusion and Deskew, production base rectangle x±100/y±35/z[1,15] m, .8 m voxel and at most 5000 random points. The callback copies its immutable values once for all B branches and records canonical array identity. Float32 input is preserved as float32. No branch re-crops or re-samples Source.

Historical input is the common Deskew service `_common.xyzi` dense float4 output BEFORE NDT Source preprocessing. A frame stores its actual cloud, point order/IDs, reference and availability times, source/dense identities, Deskew provenance and its branch-specific O pose anchor. It cannot be reconstructed from `source_after_preprocess.pcd`. Scratch is read in callback before the next header=0 request overwrites it.

For each historical scan, transform dense base points by saved O_T_base. Crop transformed coordinates at CURRENT LiDAR-update-before causal O prediction: horizontal radius 120 m, relative O z[-10,30] m. Downsample accepted coordinates into .8 m centroid voxels without the Source rectangle or 5000 cap. Transforming only a crop center into tilted base and using its z axis would be incorrect. O and independent U share gravity direction; fixed O_T_U removes only initial yaw/translation and retains initial roll/pitch in O_T_base.

Matching uses target O coordinates and source current base coordinates; initial and final transforms are O_T_base. The immediately previous actual scan becomes B0 Target regardless of preceding match success; a failed frame's O anchor is prediction and marked `predicted_anchor`. No silent last-accepted substitution. B-M and B-K have their own declared history sets, never mislabeled B0.

Production evidence: lidar_ndt/config/lidar_ndt_param.yaml:41–100; NDTComponent constructor config_file/YAML_PATH and executeFrame:833–850; NDTProcess::preprocessPointCloud:281–305; LocalMapManager::computeLocalCylinder:378–385. The 60 m clipping_radius is inactive Source-cylinder fallback, whereas active global Target radius is 120 m, z[-10,30], update movement 10 m. The research Target uses the current prediction every scan rather than inheriting an asynchronous stale snapshot center.

Common Source retains map-feedback Deskew history as explicitly approved B0. U/O posterior isolation does not make the input wholly map-independent. No current global final pose enters target construction, initialization after first frame, acceptance or local U propagation.
