#!/usr/bin/env python3
"""Offline case extraction for the Phase G1 frontend.

Python is used only to read frozen records and emit a case list; it never
solves registration. Everything downstream is the C++ binary.

Two modes:

  aw-equiv  Rebuild the exact Phase AW matcher A/B cases from the stored
            <PHASE_AW_PRIVATE>/matcher_ab_fixed/matches.jsonl seed + source
            identity, so the C++ G0 preset can be compared against the stored
            AW GICP output.

  loop44    Select frames from the frozen loop44 transactions ledger and emit
            one case per (frame, map) pair. Maps are the frozen 0105 and 1013
            targets; the seed and the region contract are identical for both,
            which is the historical "same Source, same Seed, other map"
            experiment.

Selection is by time window and by fixed stride only. No reference trajectory
is read, so no case can be selected by how well a matcher performed.
"""
import argparse
import hashlib
import json
import os
import sys

ORIGIN_NS = 1763205419863639180  # bag recorder_first_ns, verified in the audit

DEFAULT_FRAMES = "/home/ltc/flyagle/ieskf/data/phase_as_accuracy_quality/F_multilap"
DEFAULT_TARGET_0105 = "/home/ltc/flyagle/ieskf/data/inputs/map_target.bin"
DEFAULT_TARGET_1013 = (
    "/home/ltc/flyagle/analyzer/lidar_ndt/audits/"
    "ndt_source_target_resolution_20261017/inputs/map1013_leaf8.bin"
)
DEFAULT_AW_PRIVATE = (
    "/home/ltc/flyagle/ieskf-aw/data/phase_aw_native_gicp_fusion/private"
)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def offset_s(stamp_ns):
    return (int(stamp_ns) - ORIGIN_NS) * 1e-9


def region_for(seed, generation):
    """Cylinder + Z band centred on the seed, matching the frozen AR/AS contract."""
    x, y, z = seed[0][3], seed[1][3], seed[2][3]
    return {
        "enabled": True,
        "center": [x, y, z],
        "radius": 120.0,
        "low_z": z - 10.0,
        "high_z": z + 30.0,
        "generation": int(generation),
    }


def write_cases(path, cases):
    with open(path, "w") as f:
        for c in cases:
            f.write(json.dumps(c, sort_keys=True) + "\n")
    return len(cases)


def mode_aw_equiv(a):
    matches = os.path.join(a.aw_private, "matcher_ab_fixed", "matches.jsonl")
    if not os.path.exists(matches):
        sys.exit(f"missing {matches}")
    cases = []
    missing = 0
    with open(matches) as f:
        for line in f:
            if not line.strip():
                continue
            r = json.loads(line)
            ref = int(r["ref"])
            src = os.path.join(a.frames, f"{ref}_A1.bin")
            if not os.path.exists(src):
                missing += 1
                continue
            seed = r["seed"]
            cases.append(
                {
                    "case_id": f"AWEQ_{r['index']:05d}_{ref}",
                    "corpus": "phase_aw_matcher_ab_fixed",
                    "offset_s": offset_s(ref),
                    "source_path": src,
                    "target_path": a.target_0105,
                    "target_sha256": a.sha_0105,
                    "seed": seed,
                    "region": region_for(seed, r["index"]),
                    "frame_id": r["index"],
                }
            )
    n = write_cases(a.out, cases)
    print(f"aw-equiv: {n} cases, {missing} frames missing a source file")
    if missing:
        print("  NOTE: missing frames are dropped, not substituted")


def mode_loop44(a):
    ledger = os.path.join(a.frames, "transactions.jsonl")
    if not os.path.exists(ledger):
        sys.exit(f"missing {ledger}")
    rows = []
    with open(ledger) as f:
        for line in f:
            if line.strip():
                rows.append(json.loads(line))
    rows.sort(key=lambda r: r["ref"])
    print(f"loop44: ledger frames {len(rows)}")

    def in_span(r, lo, hi):
        o = offset_s(r["ref"])
        return lo <= o <= hi

    selected = {}
    for r in rows:
        o = offset_s(r["ref"])
        if 395.606 <= o <= 398.800:
            selected[r["index"]] = "W_ONSET_RECOVERY"
        elif 273.556 <= o <= 480.206 and (r["index"] % 100) == 0:
            selected[r["index"]] = "CTRL_STRIDE100"
    print(f"  selected {len(selected)} frames "
          f"({sum(1 for v in selected.values() if v == 'W_ONSET_RECOVERY')} onset/recovery, "
          f"{sum(1 for v in selected.values() if v == 'CTRL_STRIDE100')} stride control)")

    maps = [("0105", a.target_0105, a.sha_0105), ("1013", a.target_1013, a.sha_1013)]
    cases = []
    for r in rows:
        if r["index"] not in selected:
            continue
        stamp = int(r["ref"])
        src = os.path.join(a.frames, f"{stamp}_A1.bin")
        if not os.path.exists(src):
            continue
        for tag, tpath, tsha in maps:
            seed = r["seed"]
            cases.append(
                {
                    "case_id": f"L44_{r['index']:05d}_{stamp}_{tag}",
                    "corpus": selected[r["index"]],
                    "offset_s": offset_s(stamp),
                    "source_path": src,
                    "target_path": tpath,
                    "target_sha256": tsha,
                    "seed": seed,
                    "region": region_for(seed, r["index"]),
                    "frame_id": r["index"],
                    "map": tag,
                }
            )
    n = write_cases(a.out, cases)
    print(f"loop44: wrote {n} cases to {a.out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["aw-equiv", "loop44"], required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", default=DEFAULT_FRAMES)
    ap.add_argument("--target-0105", default=DEFAULT_TARGET_0105)
    ap.add_argument("--target-1013", default=DEFAULT_TARGET_1013)
    ap.add_argument("--aw-private", default=DEFAULT_AW_PRIVATE)
    a = ap.parse_args()

    for p in (a.target_0105, a.target_1013):
        if not os.path.exists(p):
            sys.exit(f"missing target: {p}")
    a.sha_0105 = sha256(a.target_0105)
    a.sha_1013 = sha256(a.target_1013)
    print(f"target 0105 sha {a.sha_0105}")
    print(f"target 1013 sha {a.sha_1013}")

    if a.mode == "aw-equiv":
        mode_aw_equiv(a)
    else:
        mode_loop44(a)


if __name__ == "__main__":
    main()
