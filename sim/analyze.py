"""Aggregate results/runs/**/*.json into results/results.json and results/results.md.

Every number quoted in the README comes from these two files.

Statistics: each scenario is run with the same 10 seeds for every controller (same arrivals,
same driver randomness), so controllers are compared by paired differences per seed. Intervals
are two-sided 95 % Student-t intervals across seeds (n = 10, df = 9).
"""
from __future__ import annotations

import json
import math
import statistics
from pathlib import Path

from common import CONFIG, RESULTS, SCENARIOS, WARMUP_S, DEMAND_END_S

T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262,
       10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131, 16: 2.120, 17: 2.110,
       18: 2.101, 19: 2.093, 20: 2.086, 25: 2.060, 30: 2.042}
CONTROLLERS = ["fixed", "actuated", "sumo"]
LABEL = {"fixed": "Fixed-time (Webster)", "actuated": "Actuated (this repo)", "sumo": "SUMO actuated (reference)"}


def t_crit(df: int) -> float:
    return T95[df] if df in T95 else T95[max(k for k in T95 if k <= df)]


def ci(xs: list[float]) -> dict:
    n = len(xs)
    m = statistics.fmean(xs)
    if n < 2:
        return {"mean": m, "lo": m, "hi": m, "n": n}
    h = t_crit(n - 1) * statistics.stdev(xs) / math.sqrt(n)
    return {"mean": m, "lo": m - h, "hi": m + h, "n": n}


def load_runs() -> list[dict]:
    return [json.loads(p.read_text(encoding="utf-8")) for p in sorted((RESULTS / "runs").rglob("*.json"))]


def per_hour(count: float) -> float:
    return count * 3600.0 / (DEMAND_END_S - WARMUP_S)


def terminations(run: dict, kind: str) -> float:
    return per_hour(sum(v[kind] for v in run["terminations"].values()))


METRICS = {
    "mean_delay_s": lambda r: r["mean_delay_s"],
    "queue_p95_veh": lambda r: r["queue_p95_worst_approach_veh"],
    "throughput_vph": lambda r: r["throughput_vph"],
    "gap_outs_per_h": lambda r: terminations(r, "gap_out"),
    "max_outs_per_h": lambda r: terminations(r, "max_out"),
}


def fmt(c: dict, digits: int = 1) -> str:
    return f"{c['mean']:.{digits}f} [{c['lo']:.{digits}f}, {c['hi']:.{digits}f}]"


def verdict(diff: dict, lower_is_better: bool = True) -> str:
    if diff["lo"] <= 0 <= diff["hi"]:
        return "no clear difference"
    better = diff["hi"] < 0 if lower_is_better else diff["lo"] > 0
    return "actuated better" if better else "actuated WORSE"


def main() -> None:
    runs = load_runs()
    compare = [r for r in runs if r["experiment"] == "compare"]
    fault = [r for r in runs if r["experiment"] == "fault"]
    ablation = [r for r in runs if r["experiment"] == "ablation"]
    cfg = json.loads(CONFIG.read_text(encoding="utf-8"))
    out: dict = {"method": __doc__.strip(), "scenarios": {}, "safety": {}, "fault": {}}
    md: list[str] = []

    # ---------------------------------------------------------------- controller comparison
    md.append("## Controller comparison\n")
    md.append(f"Means over seeds with 95 % t-intervals. Measurement window {WARMUP_S}-{DEMAND_END_S} s; "
              "delay = SUMO timeLoss + insertion delay per vehicle; queue = 95th percentile over time of "
              "the longest approach queue (halting vehicles); gap-/max-outs per hour summed over all phases.\n")
    md.append("| Scenario | veh/h per approach | Controller | Mean delay (s/veh) | p95 queue (veh) | Throughput (veh/h) | Gap-outs/h | Max-outs/h |")
    md.append("|---|---|---|---|---|---|---|---|")
    for scen in SCENARIOS:
        rows = {c: sorted((r for r in compare if r["scenario"] == scen and r["controller"] == c), key=lambda r: r["seed"])
                for c in CONTROLLERS}
        if not rows["actuated"]:
            continue
        s_out: dict = {"demand_vph_per_approach": SCENARIOS[scen], "controllers": {}, "paired": {}}
        fixed_runs = rows["fixed"]
        if fixed_runs:
            w = fixed_runs[0]["webster"]
            s_out["webster"] = {"Y": w["Y"], "cycle_s": w["cycle_s"], "optimal_cycle_s": w["optimal_cycle_s"],
                                "clamped": w["clamped"], "green_s": w["green_s"]}
        demand = "/".join(str(v) for v in dict.fromkeys(SCENARIOS[scen].values()))
        for c in CONTROLLERS:
            if not rows[c]:
                continue
            stats = {m: ci([f(r) for r in rows[c]]) for m, f in METRICS.items()}
            stats["unfinished_vehicles_total"] = sum(r["unfinished_vehicles"] for r in rows[c])
            # Through (incl. near-side turns) vs turn-across delay: who pays for the result.
            groups = rows[c][0]["mean_delay_by_group_s"].keys()
            stats["delay_through_s"] = ci([statistics.fmean([r["mean_delay_by_group_s"][g] for g in groups if g.endswith("_T")]) for r in rows[c]])
            stats["delay_turn_across_s"] = ci([statistics.fmean([r["mean_delay_by_group_s"][g] for g in groups if g.endswith("_R")]) for r in rows[c]])
            stats["never_inserted_total"] = sum(r["vehicles_never_inserted"] for r in rows[c])
            s_out["controllers"][c] = stats
            go = "-" if c == "fixed" else f"{stats['gap_outs_per_h']['mean']:.0f}"
            mo = "-" if c == "fixed" else f"{stats['max_outs_per_h']['mean']:.0f}"
            md.append(f"| {scen} | {demand} | {LABEL[c]} | {fmt(stats['mean_delay_s'])} | {fmt(stats['queue_p95_veh'])} | "
                      f"{fmt(stats['throughput_vph'], 0)} | {go} | {mo} |")
        for other in ("fixed", "sumo"):
            if len(rows[other]) != len(rows["actuated"]):
                continue
            assert [r["seed"] for r in rows[other]] == [r["seed"] for r in rows["actuated"]]
            pd = {}
            for m in ("mean_delay_s", "queue_p95_veh", "throughput_vph"):
                pd[m] = ci([METRICS[m](a) - METRICS[m](b) for a, b in zip(rows["actuated"], rows[other])])
                pd[m]["verdict"] = verdict(pd[m], lower_is_better=m != "throughput_vph")
            pd["mean_delay_pct"] = 100.0 * pd["mean_delay_s"]["mean"] / s_out["controllers"][other]["mean_delay_s"]["mean"]
            s_out["paired"][f"actuated_minus_{other}"] = pd
        out["scenarios"][scen] = s_out

    md.append("\n### Paired differences (actuated minus the other controller, same seeds)\n")
    md.append("| Scenario | vs | Delay diff (s/veh) | Delay diff (%) | p95 queue diff (veh) | Throughput diff (veh/h) | Delay verdict |")
    md.append("|---|---|---|---|---|---|---|")
    for scen, s_out in out["scenarios"].items():
        for key, pd in s_out["paired"].items():
            other = key.split("_minus_")[1]
            md.append(f"| {scen} | {LABEL[other]} | {fmt(pd['mean_delay_s'])} | {pd['mean_delay_pct']:+.1f} | "
                      f"{fmt(pd['queue_p95_veh'])} | {fmt(pd['throughput_vph'], 0)} | {pd['mean_delay_s']['verdict']} |")

    md.append("\n### Delay by movement type\n")
    md.append("Mean delay of through traffic (with near-side turns) and of turn-across traffic, averaged over the four approaches.\n")
    md.append("| Scenario | Controller | Through (s/veh) | Turn-across (s/veh) |")
    md.append("|---|---|---|---|")
    for scen, s_out in out["scenarios"].items():
        for c, st in s_out["controllers"].items():
            md.append(f"| {scen} | {LABEL[c]} | {fmt(st['delay_through_s'])} | {fmt(st['delay_turn_across_s'])} |")

    md.append("\n### Webster plans used by the fixed-time controller\n")
    md.append("| Scenario | Y | C0 (s) | Cycle used (s) | Greens (s) |")
    md.append("|---|---|---|---|---|")
    for scen, s_out in out["scenarios"].items():
        w = s_out.get("webster")
        if w:
            c0 = "inf" if w["optimal_cycle_s"] is None else f"{w['optimal_cycle_s']:.0f}"
            greens = ", ".join(f"{k} {v:g}" for k, v in w["green_s"].items())
            md.append(f"| {scen} | {w['Y']:.2f} | {c0} | {w['cycle_s']:g}{' (clamped)' if w['clamped'] else ''} | {greens} |")

    # ---------------------------------------------------------------- stop-bar ablation
    if ablation:
        out["ablation_stopbar_extend"] = {}
        md.append("\n## Ablation: stop-bar loops that extend the green\n")
        md.append("The configuration uses the stop-bar loops of the turn-across lanes as call-only. This compares it with "
                  "the same controller when those loops also extend the green (paired by seed).\n")
        md.append("| Scenario | Call-only (config) delay (s/veh) | Extending delay (s/veh) | Difference extend - call (s/veh) |")
        md.append("|---|---|---|---|")
        for scen in SCENARIOS:
            base = sorted((r for r in compare if r["scenario"] == scen and r["controller"] == "actuated"), key=lambda r: r["seed"])
            alt = sorted((r for r in ablation if r["scenario"] == scen), key=lambda r: r["seed"])
            if not alt or len(alt) != len(base):
                continue
            assert [r["seed"] for r in alt] == [r["seed"] for r in base]
            a = {"call_only": ci([r["mean_delay_s"] for r in base]), "extend": ci([r["mean_delay_s"] for r in alt]),
                 "extend_minus_call": ci([x["mean_delay_s"] - y["mean_delay_s"] for x, y in zip(alt, base)])}
            out["ablation_stopbar_extend"][scen] = a
            md.append(f"| {scen} | {fmt(a['call_only'])} | {fmt(a['extend'])} | {fmt(a['extend_minus_call'])} |")

    # ---------------------------------------------------------------- safety
    md.append("\n## Safety invariants\n")
    md.append("Violations found by the independent auditor in the signal states SUMO actually displayed, and commands "
              "refused by the safety guard, summed over all runs.\n")
    md.append("| Controller | Runs | Auditor violations | Guard refusals | Runs ending in fail-safe |")
    md.append("|---|---|---|---|---|")
    for c in CONTROLLERS:
        rs = [r for r in runs if r["controller"] == c]
        if not rs:
            continue
        s = {"runs": len(rs), "auditor_violations": sum(r["invariant_violations"] for r in rs),
             "inconsistent_group_states": sum(r["inconsistent_group_states"] for r in rs),
             "guard_refusals": sum(r.get("guard_refusals", 0) for r in rs) if c != "sumo" else None,
             "failsafe_runs": sum(bool(r.get("failsafe")) for r in rs) if c != "sumo" else None}
        out["safety"][c] = s
        na = lambda v: "n/a" if v is None else str(v)  # noqa: E731
        md.append(f"| {LABEL[c]} | {s['runs']} | {s['auditor_violations']} | {na(s['guard_refusals'])} | {na(s['failsafe_runs'])} |")

    # ---------------------------------------------------------------- detector fault
    if fault:
        md.append("\n## Detector fault: all four north-south turn-across loops stuck off from 600 s (medium demand)\n")
        stuck_off = cfg["faults"]["stuck_off_s"]
        md.append(f"Fault threshold `stuck_off_s` = {stuck_off} s, fallback recall = `{cfg['faults']['fallback_recall']}`. "
                  "Turn-across movements N->W and S->E (groups N_R, S_R) are the ones served by the faulty loops.\n")
        md.append("| Variant | Mean delay, all vehicles (s) | Mean delay, N_R + S_R (s) | Vehicles unfinished at end | Fault declared at (s) |")
        md.append("|---|---|---|---|---|")
        for variant in ("no_fault", "fault_handling_off", "fault_handling_on"):
            rs = sorted((r for r in fault if r["variant"] == variant), key=lambda r: r["seed"])
            if not rs:
                continue
            turn = [statistics.fmean([r["mean_delay_by_group_s"]["N_R"], r["mean_delay_by_group_s"]["S_R"]]) for r in rs]
            declared = sorted({f["at_s"] for r in rs for f in r.get("faults_declared", [])})
            bins = [statistics.fmean(r["greens_started_per_300s"]["NS_right"][i] for r in rs)
                    for i in range(len(rs[0]["greens_started_per_300s"]["NS_right"]))]
            v = {"runs": len(rs), "mean_delay_s": ci([r["mean_delay_s"] for r in rs]), "turn_across_ns_delay_s": ci(turn),
                 "unfinished_total": sum(r["unfinished_vehicles"] for r in rs), "fault_declared_at_s": declared,
                 "faults_declared": sorted({(f["detector"], f["kind"]) for r in rs for f in r.get("faults_declared", [])}),
                 "ns_right_greens_per_300s_mean": bins,
                 "sim_end_s_max": max(r["sim_end_s"] for r in rs)}
            out["fault"][variant] = v
            when = f"{min(declared):g} to {max(declared):g}" if declared else "-"
            md.append(f"| {variant} | {fmt(v['mean_delay_s'])} | {fmt(v['turn_across_ns_delay_s'])} | {v['unfinished_total']} | {when} |")
        md.append("\nNS_right greens started per 300 s bin (mean over seeds; bins start at 0 s):\n")
        md.append("| Variant | " + " | ".join(f"{300 * i}" for i in range(13)) + " |")
        md.append("|---|" + "---|" * 13)
        for variant, v in out["fault"].items():
            md.append(f"| {variant} | " + " | ".join(f"{x:.1f}" for x in v["ns_right_greens_per_300s_mean"][:13]) + " |")

    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "results.json").write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    (RESULTS / "results.md").write_text("# Results\n\nGenerated by `sim/analyze.py` from `results/runs/`. Do not edit by hand.\n\n"
                                        + "\n".join(md) + "\n", encoding="utf-8")
    print("\n".join(md))


if __name__ == "__main__":
    main()
