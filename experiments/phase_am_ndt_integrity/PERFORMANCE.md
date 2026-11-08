# Measured performance

CPU geometry diagnostics on284 frames: median6.603ms,p957.205ms,max8.442ms; full Target tree build150.851ms. This excludes raw Deskew and full CUDA pipeline.

35 actual CUDA request/response times: median3.626ms,p954.753ms,max198.600ms. Seven-seed batch wall times measured directly:223.394,27.759,22.360,22.160,23.574ms. First batch includes service/map/device startup. Backend align wall median3.297ms,max4.608ms. Cold batch exceeds50ms; warm batches fit the isolated budget. No end-to-end20Hz claim: ROS queueing,Deskew,global management and actual live contention were not measured together. No timing estimated by multiplying historical1ms.
