# Independent validation

NOT EXECUTED as an independent accuracy test: no qualified independent map/Source/reference covers the original first frame. An alternate PCD without acquisition/datum evidence cannot supply independent truth. Earlier/later reference time ranges do not contain the frozen event. No temporal extrapolation, delay optimization, alignment or chosen best seed was used.

The5m endpoint has a local source-plane association witness and a conditional stationary solution. All map/Source rows used there come from the same generation scene. G1/G2 and AA partial were also generated from that scene; comparing them is self-consistency, not held-out validation. Authorized relative pose-distance aggregates in key_results.json quantify their disagreement but cannot select a correct candidate. AF's stored candidate surface residuals remain historical shared-data evidence, not a new independent score.

Next independent validation needs repeated captures with known scan membership, fixed frame/datum/extrinsics and a separately verified motion/pose reference, plus physically linked surface groups. Training and validation must exclude overlapping source observations and map fit neighborhoods or explicitly model their common error. No absolute position/attitude accuracy improvement or independent covariance calibration is claimed.
