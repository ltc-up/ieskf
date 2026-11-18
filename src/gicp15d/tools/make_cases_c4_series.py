#!/usr/bin/env python3
"""Emit frozen EKF-seed cases from the FAR-LIO phase_bg C4 series.

Each case uses the REAL historical inputs: Source, Target and the EKF
pre-match predicted Seed, all frozen per frame in
phase_bg_far_lio/c4/frozen/<NN>/. The engineering proxy and the historical
CUDA-NDT pose come from WALL_frames.csv (ndt_x_good/ndt_y_good and ndt_x/ndt_y),
which is the exact table the phase_e4 analysis used to define proxy_error.

Offline extraction only; the C++ frontend does every registration.
"""
import argparse, csv, json, os

FROZEN = "/media/ltc/新加卷/autoeagle/phase_bg_far_lio/c4/frozen"
WALL = ("/home/ltc/flyagle/analyzer/2026-10-05_ndt_localization_final_archive/"
        "analysis/tent_map_hypothesis/WALL_frames.csv")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--include-duplicates", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    req = json.load(open(os.path.join(FROZEN, "requests.json")))
    wall = {int(r["sensor_stamp_ns"]): r for r in csv.DictReader(open(WALL))}

    seen, cases, meta = set(), [], []
    for e in sorted(req, key=lambda x: x["header"]):
        stamp = int(e["header"])
        if not a.include_duplicates and stamp in seen:
            continue
        seen.add(stamp)
        w = wall.get(stamp)
        proxy = None if w is None else (float(w["ndt_x_good"]), float(w["ndt_y_good"]))
        ndt = None if w is None else (float(w["ndt_x"]), float(w["ndt_y"]))
        seed = e["seed_common"]
        cases.append({
            "case_id": f"C4_{e['case_id']}",
            "corpus": "C4_EKF_SEED_SERIES",
            **({"offset_s": float(w["t"])} if w else {}),
            "source_path": e["source_path"],
            "target_path": e["target_path"],
            "target_sha256": e["target_sha"],
            "seed": seed,
            "region": {"enabled": True, "center": [seed[0][3], seed[1][3], seed[2][3]],
                       "radius": 120.0, "low_z": seed[2][3] - 10.0,
                       "high_z": seed[2][3] + 30.0, "generation": 1},
            "frame_id": int(e["case_id"].split("_")[1]),
        })
        meta.append({
            "case_id": e["case_id"], "stamp_ns": stamp,
            **({"offset_s": float(w["t"])} if w else {}),
            "source_path": e["source_path"], "target_path": e["target_path"],
            "target_sha": e["target_sha"],
            "seed_xyz": [seed[0][3], seed[1][3], seed[2][3]],
            "proxy": proxy, "historical_ndt": ndt,
            "historical_fitness": None if not w else w.get("fitness_score"),
            "historical_match_score": None if not w else w.get("match_score"),
            "historical_good_diff_m": None if not w else float(w["good_diff"]),
            "historical_reject": None if not w else w.get("reject"),
            "historical_seed_lateral_m": None if not w else float(w["seed_lateral"]),
            "target_points": None if not w else int(w["target_points"]),
            "map_version": None if not w else w.get("map_version"),
        })

    with open(os.path.join(a.out, "cases_c4_series.jsonl"), "w") as f:
        for c in cases:
            f.write(json.dumps(c, sort_keys=True) + "\n")
    json.dump({"n": len(cases), "distinct_headers": len(seen), "cases": meta},
              open(os.path.join(a.out, "c4_series_identity.json"), "w"),
              indent=1, sort_keys=True)
    n_proxy = sum(1 for m in meta if m["proxy"])
    print(f"{len(cases)} cases, {len(seen)} distinct headers, {n_proxy} with a proxy row")


if __name__ == "__main__":
    main()
