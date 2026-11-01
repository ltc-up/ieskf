# Corrected controlled operators

A is unchanged full compensation. B removes ONLY the world-Z translation term actually supplied to the kernel: `p_baseZ(t)-p_baseZ(ref)=0`. It does not alter raw point Z, any quaternion, reference pose, XY translation or T_base_lidar. All rotation of the point AND the LiDAR extrinsic lever remains in `R(t) T_base_lidar p`. C keeps the same p_base(t) and uses `R_C(t)=Rz(yaw(t)-yaw_ref) R_ref`, fixing reference Roll/Pitch while retaining time-varying yaw; no Euler angles are zeroed and R_C(ref)=R_ref. This hybrid is causal diagnosis, not physically recommended deskew.

Important reference-point boundary: the upstream base pose already includes IMU→base transport. Its Z term is part of the explicit kernel translation removed by B, not separately recomputed. The FIRST attempted B instead removed IMU-origin relative Z and retained that transport term. That did NOT equal the requested kernel operator. It is archived as DEFINITION_ERROR and excluded from all primary tables. Corrected kernel A/C reproduced their earlier bytes exactly, so their existing matches were reused; only5 affected B full fits/updates were repeated. No production file changed.

All groups share raw input point count/order/implicit(sensor,index) IDs, intensity, time, reference, extrinsics and one double-intermediate/float32-output CUDA path. Reference-time transforms are identity on base points (the static extrinsic still applies to raw LiDAR points), checked by5 new operator tests. An independent CPU formula reproduces GPU float outputs exactly on all recorded cases. A additionally reproduces the original executable and frozen AA Sources exactly. These are numerical equivalence checks, not height accuracy.

|Event|Difference|Raw points|abs dZ P50 m|P95 m|P99 m|Max m|
|---|---|---:|---:|---:|---:|---:|
|E1|B-A|124819|0.034652|0.060301|0.062845|0.063553|
|E1|C-A|124819|0.011592|0.049342|0.097026|0.525082|
|E2|B-A|124999|0.034442|0.060300|0.062908|0.063640|
|E2|C-A|124999|0.005347|0.039124|0.059261|0.125118|
|E3|B-A|125661|0.036606|0.063115|0.065750|0.066496|
|E3|C-A|125661|0.006515|0.038671|0.059856|0.102427|
|E4|B-A|113010|0.016261|0.024885|0.025981|0.026287|
|E4|C-A|113010|0.069737|0.427729|1.007161|2.173410|
|E5|B-A|94505|0.011711|0.016597|0.017100|0.017207|
|E5|C-A|94505|0.009876|0.060934|0.141294|0.382619|

Signed XYZ differences and private ranges/structure are retained locally; public key_results.json contains absolute XYZ quantiles too. B-A is a world-vertical translation difference rotated into the reference-body frame, so its reference-body X/Y components need not be zero. C includes range-dependent angular effects: the large-RP E4 has P99~1.007m and max~2.173m raw Z changes. This is point-coordinate sensitivity, not a true ground-height error.

The SAME historical crop→voxel0.8m→deterministic sample-cap pipeline is applied in the SAME common frame, followed by the same native rebase. Raw point identities remain one-to-one; downstream voxel memberships/centroids and source counts can change as a consequence. Thus NDT effects include this fixed pipeline's response, not a frozen-voxel-only experiment. Max far-point differences may be cropped out or averaged down. No statistical correlation law is fitted from5 cases; relation to speed/RP is descriptive, and no cloud is selected by visual smoothness.
