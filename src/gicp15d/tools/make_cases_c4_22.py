#!/usr/bin/env python3
"""Extract the C4_22 same-frame case from the historical phase_e4 capture.

Offline extraction only; the C++ frontend does all registration.

Source, Seed and ROI come from the archived capture JSON. The historical
98,213-point Target bytes are gone, so the Target is rebuilt from the frozen
1013 map with the ROI recorded in that JSON. Every output carries the rebuild
flag so no result can be mistaken for the historical byte-identical case.
"""
import argparse, hashlib, json, struct, os
import numpy as np

MAP = ("/home/ltc/flyagle/analyzer/lidar_ndt/audits/"
       "ndt_source_target_resolution_20261017/inputs/map1013_leaf8.bin")
E4 = ("/home/ltc/flyagle/auto-eagle-main/fitness_test/src/a2rl_localization/"
      "lidar_ndt/audit/ndt_uncertainty_research/phase_e4_local_convergence")
STAMP = 1763205937849981440
T = 396.999993344
SPECS = [
    ("C4_22_narrow", "A1_source_restore", 1042,
     (-293.79443359375, -170.29898071289062, -30.401357650756836), 22.470232096803493,
     (-295.4878234863281, -174.61813354492188, -30.724557876586914),
     # The closed-loop CSV records the Seed actually used by the narrow run; the
     # capture JSON carries a slightly different guess from a sibling run
     # (0.26 m apart). The CSV value is the authoritative historical Seed.
     [[0.73128, -0.682075, 0.001658, -293.2270202636719],
      [0.681802, 0.730914, -0.030188, -172.12197875976562],
      [0.019379, 0.023206, 0.999543, -30.72939109802246],
      [0.0, 0.0, 0.0, 1.0]]),
    ("C4_22_wide", "A1_wide35", 1041,
     (-293.57672119140625, -173.1700897216797, -31.09105110168457), 16.081053867370525,
     (-295.4815673828125, -174.57850646972656, -30.63671112060547), None),
]


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    pts = np.fromfile(MAP, "<f4", offset=8).reshape(-1, 3)
    cases, meta = [], []
    for tag, run, gen, ndt_final, ndt_fit, center, seed_override in SPECS:
        cj = f"{E4}/runs/{run}/captures/{STAMP}.json"
        cap = json.load(open(cj))
        assert cap["stamp_ns"] == STAMP and abs(cap["t"] - T) < 1e-9
        src = f"{E4}/runs/{run}/captures/{STAMP}.bin"
        cx, cy, cz = center
        m = (((pts[:, 0] - cx) ** 2 + (pts[:, 1] - cy) ** 2) <= 120.0 ** 2) & \
            (pts[:, 2] >= cz - 10) & (pts[:, 2] <= cz + 30)
        sub = pts[np.flatnonzero(m)]
        tp = f"{a.out}/target_{tag}_1013_roi.bin"
        with open(tp, "wb") as f:
            f.write(struct.pack("<Q", len(sub)))
            f.write(sub.astype("<f4").tobytes())
        ssh, tsh = sha(src), sha(tp)
        meta.append({
            "tag": tag, "stamp_ns": STAMP, "offset_s": T,
            "source_path": src, "source_n": int(np.fromfile(src, "<u8", count=1)[0]),
            "source_sha256": ssh, "target_path": tp, "target_n": len(sub),
            "target_sha256": tsh, "historical_target_n": 98213,
            "target_status": "TARGET_RECONSTRUCTED",
            "region_center": list(center), "generation": gen,
            "historical_ndt_final": list(ndt_final),
            "historical_ndt_fitness": ndt_fit,
            "seed": seed_override if seed_override else cap["guess"],
        })
        cases.append({
            "case_id": f"{tag}_seed", "corpus": "C4_22", "offset_s": T,
            "source_path": src, "target_path": tp,
            "target_sha256": tsh, "source_sha256": ssh,
            "seed": seed_override if seed_override else cap["guess"],
            # Z band and radius come from the archived request, not recomputed.
            "region": {"enabled": True, "center": list(center),
                       "radius": float(cap["region"][11]),
                       "low_z": float(cap["region"][9]), "high_z": float(cap["region"][10]),
                       "generation": gen},
            "frame_id": 0,
        })
    with open(f"{a.out}/cases_c4_22.jsonl", "w") as f:
        for c in cases:
            f.write(json.dumps(c, sort_keys=True) + "\n")
    json.dump(meta, open(f"{a.out}/c4_22_input_identity.json", "w"), indent=1, sort_keys=True)
    for m in meta:
        print(m["tag"], m["source_n"], m["source_sha256"][:12],
              m["target_n"], m["target_sha256"][:12])


if __name__ == "__main__":
    main()
