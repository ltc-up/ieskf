#!/usr/bin/env python3
"""Aggregate Phase G1 frontend output. Offline statistics only.

Definitions used in the report and in this script:

  d_seed   ||T_final.xy - seed.xy||           how far the solver moved
  d_ndt    ||T_final.xy - T_cuda_ndt.xy||     matcher agreement with the stored
                                              CUDA-NDT baseline. This is a
                                              PROXY, not ground truth.
  d_map    ||T_final.xy(0105) - T_final(1013)||  for the same frame, seed and
                                              preset: how much the map changes
                                              the answer.

No reference trajectory participates in any selection.
"""
import argparse
import collections
import json
import math
import os
import statistics as st

WINDOW_ORDER = ["W_ONSET_RECOVERY", "CTRL_STRIDE100"]
PRESET_ORDER = [
    "G0_AW_EQUIV", "G1_SPECTRUM", "G1_ROI_CROP", "G1_NORMAL_GATE",
    "G1_RECIPROCAL", "G1_ROBUST_GM", "G1_FULL", "G1_COARSE",
]


def load_results(path):
    out = []
    with open(path) as f:
        for line in f:
            if line.strip():
                out.append(json.loads(line))
    return out


def load_ndt_baseline(frames_dir):
    """frame_id -> (nderived x, y, converged) from the stored CUDA-NDT ledger."""
    base = {}
    path = os.path.join(frames_dir, "transactions.jsonl")
    if not os.path.exists(path):
        return base
    with open(path) as f:
        for line in f:
            if not line.strip():
                continue
            r = json.loads(line)
            ndt = r.get("ndt") or {}
            fin = ndt.get("final")
            if not fin:
                continue
            base[int(r["index"])] = {
                "x": fin[0][3],
                "y": fin[1][3],
                "converged": bool(ndt.get("converged")),
                "valid": bool(ndt.get("candidate_valid")),
                "fitness": ndt.get("fitness"),
            }
    return base


def pct(vals, p):
    if not vals:
        return float("nan")
    v = sorted(vals)
    i = min(len(v) - 1, int(math.floor(p * (len(v) - 1))))
    return v[i]


def summarise(vals):
    if not vals:
        return {"n": 0}
    return {
        "n": len(vals),
        "mean": st.fmean(vals),
        "p50": pct(vals, 0.50),
        "p95": pct(vals, 0.95),
        "max": max(vals),
        "min": min(vals),
    }


def classify(d):
    if d is None or not math.isfinite(d):
        return "UNKNOWN"
    if d <= 0.5:
        return "AGREES_NDT"
    if d <= 2.0:
        return "NEAR_NDT"
    if d <= 4.0:
        return "DISPLACED"
    return "DIFFERENT_BASIN"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", required=True)
    ap.add_argument("--frames", required=True)
    ap.add_argument("--out-json", required=True)
    ap.add_argument("--out-md", required=True)
    ap.add_argument("--title", default="Phase G1 validation")
    a = ap.parse_args()

    results = load_results(a.results)
    ndt = load_ndt_baseline(a.frames)

    by_key = collections.defaultdict(list)
    # case_id encodes L44_<ordinal>_<stamp>_<map>; frame_id is not duplicated in
    # the payload, so recover it from the frozen case identity.
    for r in results:
        cid = r["case_id"]
        parts = cid.split("_")
        r["frame_id"] = int(parts[1])
        r["map"] = "1013" if cid.endswith("_1013") else "0105"
        by_key[(r["preset"], r.get("corpus"), r["map"])].append(r)

    # d_map pairs
    d_map = collections.defaultdict(list)
    index = {}
    for r in results:
        index[(r["preset"], r["frame_id"], r["map"])] = r
    for (preset, fid, m), r in index.items():
        if m != "0105":
            continue
        other = index.get((preset, fid, "1013"))
        if not other:
            continue
        if not (r["candidate_valid"] and other["candidate_valid"]):
            continue
        dd = math.hypot(r["final"][0][3] - other["final"][0][3],
                        r["final"][1][3] - other["final"][1][3])
        d_map[(preset, r.get("corpus"))].append(dd)

    agg = {}
    for (preset, corpus, m), rows in sorted(by_key.items(),
                                            key=lambda kv: (PRESET_ORDER.index(kv[0][0])
                                                            if kv[0][0] in PRESET_ORDER else 99,
                                                            str(kv[0][1]), kv[0][2])):
        d_seed, d_ndt, d_rot = [], [], []
        classes = collections.Counter()
        exits = collections.Counter()
        pairs, cond, align_ms, diag_ms = [], [], [], []
        sectors, inlier_ratio = [], []
        for r in rows:
            exits[r["exit_reason"].split(":")[0]] += 1
            if not r["candidate_valid"]:
                continue
            dx = r["final"][0][3] - r["seed"][0][3]
            dy = r["final"][1][3] - r["seed"][1][3]
            d_seed.append(math.hypot(dx, dy))
            d_rot.append(math.degrees(r["seed_rotation_angle"]))
            base = ndt.get(r["frame_id"])
            if base and base["valid"]:
                dd = math.hypot(r["final"][0][3] - base["x"], r["final"][1][3] - base["y"])
                d_ndt.append(dd)
                classes[classify(dd)] += 1
            pairs.append(r["residuals"]["pairs"])
            inlier_ratio.append(r["residuals"].get("inlier_ratio", float("nan")))
            sectors.append(r["residuals"]["sectors_occupied"])
            cond.append(r["information"]["condition_number"])
            align_ms.append(r["timing"]["align_ms"])
            diag_ms.append(r["timing"]["total_ms"])
        agg[f"{preset}|{corpus}|{m}"] = {
            "preset": preset,
            "corpus": corpus,
            "map": m,
            "frames": len(rows),
            "converged": sum(1 for r in rows if r["converged"]),
            "candidate_valid": sum(1 for r in rows if r["candidate_valid"]),
            "exit_reasons": dict(exits),
            "d_seed": summarise(d_seed),
            "d_seed_rot_deg": summarise(d_rot),
            "d_ndt_proxy": summarise(d_ndt),
            "ndt_agreement_classes": dict(classes),
            "pairs": summarise(pairs),
            "inlier_ratio": summarise([v for v in inlier_ratio if math.isfinite(v)]),
            "sectors_occupied": summarise(sectors),
            "condition_number_median": pct(cond, 0.5),
            "align_ms": summarise(align_ms),
            "total_ms": summarise(diag_ms),
        }

    payload = {
        "results_file": a.results,
        "frames_dir": a.frames,
        "rows": len(results),
        "aggregate": agg,
        "d_map": {f"{p}|{c}": summarise(v) for (p, c), v in d_map.items()},
    }
    with open(a.out_json, "w") as f:
        json.dump(payload, f, indent=1, sort_keys=True)

    # ---------------- markdown ----------------
    lines = [f"# {a.title}: aggregate", ""]
    lines.append(f"rows={len(results)}  frames={len(ndt)}  aggregates={len(agg)}")
    lines.append("")
    lines.append("`d_ndt` is matcher agreement with the stored CUDA-NDT pose, "
                 "**a proxy, not ground truth**.")
    lines.append("")
    for corpus in WINDOW_ORDER:
        keys = [k for k in agg if agg[k]["corpus"] == corpus]
        if not keys:
            continue
        lines.append(f"## {corpus}")
        lines.append("")
        lines.append("| preset | map | frames | valid | d_seed p50/p95 (m) | d_ndt p50/p95 (m) "
                     "| classes | pairs p50 | cond p50 | align p50/p95 (ms) | total p50 (ms) |")
        lines.append("|---|---|---:|---:|---|---|---|---:|---:|---|---:|")
        for preset in PRESET_ORDER:
            for m in ("0105", "1013"):
                k = f"{preset}|{corpus}|{m}"
                if k not in agg:
                    continue
                e = agg[k]
                cl = ",".join(f"{c}:{n}" for c, n in sorted(e["ndt_agreement_classes"].items()))
                lines.append(
                    f"| {preset} | {m} | {e['frames']} | {e['candidate_valid']} "
                    f"| {e['d_seed']['p50']:.3f}/{e['d_seed']['p95']:.3f} "
                    f"| {e['d_ndt_proxy'].get('p50', float('nan')):.3f}/"
                    f"{e['d_ndt_proxy'].get('p95', float('nan')):.3f} "
                    f"| {cl or '-'} | {e['pairs']['p50']:.0f} "
                    f"| {e['condition_number_median']:.3g} "
                    f"| {e['align_ms']['p50']:.2f}/{e['align_ms']['p95']:.2f} "
                    f"| {e['total_ms']['p50']:.1f} |")
        lines.append("")
        keys_dm = [k for k in payload["d_map"] if k.endswith(f"|{corpus}")]
        if keys_dm:
            lines.append(f"### map differential `d_map` ({corpus})")
            lines.append("")
            lines.append("| preset | n | p50 | p95 | max |")
            lines.append("|---|---:|---:|---:|---:|")
            for k in sorted(keys_dm, key=lambda x: PRESET_ORDER.index(x.split("|")[0])
                            if x.split("|")[0] in PRESET_ORDER else 99):
                s = payload["d_map"][k]
                lines.append(f"| {k.split('|')[0]} | {s['n']} | {s['p50']:.3f} "
                             f"| {s['p95']:.3f} | {s['max']:.3f} |")
            lines.append("")
    with open(a.out_md, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {a.out_json} and {a.out_md}")


if __name__ == "__main__":
    main()
