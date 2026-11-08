#!/usr/bin/env python3
"""Check that the numbers in README.md match results/results.json.

Covered: the summary at the top, the main results table, the paired-difference table, the
safety table, the Webster cycles quoted in the text and the saturation figures in "Where
actuated control does not win". Not covered: the detector-fault and ablation tables.

Exit 0 when every checked number matches, 1 otherwise (each mismatch is listed).

    python3 scripts/check_readme_numbers.py [--readme README.md] [--results results/results.json]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ["low", "medium", "high", "asymmetric", "saturated"]
CONTROLLERS = {  # README label prefix -> results.json key
    "Fixed-time": "fixed",
    "Actuated (this repo)": "actuated",
    "SUMO actuated, call-based skip": "sumo",
    "SUMO actuated, no skip": "sumo_noskip",
}


def ci(c: dict, digits: int = 1) -> str:
    return f"{c['mean']:.{digits}f} [{c['lo']:.{digits}f}, {c['hi']:.{digits}f}]"


def signed_ci(c: dict) -> str:
    return f"{c['mean']:+.1f} [{c['lo']:.1f}, {c['hi']:.1f}]"


def table_rows(text: str, header_start: str) -> list[list[str]]:
    """Data rows of the first markdown table whose header line starts with header_start."""
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if line.startswith(header_start):
            rows = []
            for row in lines[i + 2:]:
                if not row.startswith("|"):
                    break
                rows.append([c.strip().replace("**", "") for c in row.strip().strip("|").split("|")])
            return rows
    return []


def check(readme: str, res: dict) -> list[str]:
    errors: list[str] = []
    sc = res["scenarios"]

    def need(snippet: str, what: str) -> None:
        if snippet not in readme:
            errors.append(f"{what}: README does not contain {snippet!r}")

    # Main results table.
    seen = set()
    scenario = None
    for cells in table_rows(readme, "| Scenario (veh/h per approach) | Controller |"):
        if cells[0]:
            scenario = cells[0].split(" ")[0]
        key = next((v for k, v in CONTROLLERS.items() if cells[1].startswith(k)), None)
        if scenario not in sc or key is None:
            errors.append(f"main table: unrecognised row {cells[:2]}")
            continue
        c = sc[scenario]["controllers"][key]
        fixed = key == "fixed"
        expected = [ci(c["mean_delay_s"]), ci(c["queue_p95_veh"]), ci(c["throughput_vph"], 0),
                    "-" if fixed else f"{c['gap_outs_per_h']['mean']:.0f}",
                    "-" if fixed else f"{c['max_outs_per_h']['mean']:.0f}"]
        if cells[2:7] != expected:
            errors.append(f"main table {scenario}/{key}: README {cells[2:7]} != results.json {expected}")
        seen.add((scenario, key))
    if len(seen) != len(SCENARIOS) * len(CONTROLLERS):
        errors.append(f"main table: expected {len(SCENARIOS) * len(CONTROLLERS)} rows, found {len(seen)}")

    # Paired-difference table.
    rows = table_rows(readme, "| Scenario | vs fixed-time | vs SUMO, call-based skip |")
    if [r[0] for r in rows] != SCENARIOS:
        errors.append(f"paired table: scenarios {[r[0] for r in rows]} != {SCENARIOS}")
    for cells in rows:
        if cells[0] not in sc:
            continue
        p = sc[cells[0]]["paired"]
        expected = [f"{signed_ci(p[f'actuated_minus_{o}']['mean_delay_s'])} "
                    f"({p[f'actuated_minus_{o}']['mean_delay_pct']:+.1f} %)"
                    for o in ("fixed", "sumo", "sumo_noskip")]
        if cells[1:4] != expected:
            errors.append(f"paired table {cells[0]}: README {cells[1:4]} != results.json {expected}")

    # Safety table.
    safety = res["safety"]
    rows = table_rows(readme, "| Controller | Runs | Auditor violations |")
    if len(rows) != 4:
        errors.append(f"safety table: expected 4 rows, found {len(rows)}")
    for cells in rows:
        key = next((v for k, v in CONTROLLERS.items() if cells[0].startswith(k)), None)
        if key is None:
            errors.append(f"safety table: unrecognised row {cells[0]!r}")
            continue
        s = safety[key]
        fmt = lambda v: "n/a" if v is None else str(v)  # noqa: E731
        expected = [str(s["runs"]), str(s["auditor_violations"]), fmt(s["guard_refusals"]),
                    fmt(s["failsafe_runs"])]
        if cells[1:5] != expected:
            errors.append(f"safety table {key}: README {cells[1:5]} != results.json {expected}")

    # Summary at the top of the README.
    total_runs = sum(s["runs"] for s in safety.values())
    total_viol = sum(s["auditor_violations"] for s in safety.values())
    pf = {s: sc[s]["paired"]["actuated_minus_fixed"] for s in SCENARIOS}
    need(f"({total_runs} SUMO runs, 10 paired seeds per scenario", "summary run count")
    need(f"by {-pf['low']['mean_delay_s']['mean']:.1f} s/veh\n({-pf['low']['mean_delay_pct']:.1f} %) at low demand",
         "summary low-demand gain")
    need(f"and {-pf['asymmetric']['mean_delay_s']['mean']:.1f} s/veh "
         f"({-pf['asymmetric']['mean_delay_pct']:.1f} %) at asymmetric demand", "summary asymmetric gain")
    need(f"is {pf['saturated']['mean_delay_s']['mean']:.1f} s/veh ({pf['saturated']['mean_delay_pct']:.1f} %)\nworse at saturation",
         "summary saturation loss")
    need(f"recorded {total_viol} violations in all {total_runs} runs", "summary auditor")
    sumo_delay = [sc[s]["paired"]["actuated_minus_sumo"]["mean_delay_s"] for s in SCENARIOS]
    if not all(d["lo"] <= 0 <= d["hi"] for d in sumo_delay):
        errors.append("summary says no measurable mean-delay difference vs SUMO, but an interval excludes zero")

    # Webster plans quoted in the text.
    w = {s: sc[s]["webster"] for s in SCENARIOS}
    need(f"e.g. {w['low']['cycle_s']:.0f} s used against an optimum of {w['low']['optimal_cycle_s']:.0f} s",
         "Webster low cycle")
    need(f"at saturation the optimum ({w['saturated']['optimal_cycle_s']:.0f} s) is", "Webster saturated optimum")
    g = w["saturated"]["green_s"]
    need(f"({g['NS_through']:g}/{g['NS_right']:g}/{g['EW_through']:g}/{g['EW_right']:g} s)",
         "Webster saturated splits")

    # "Where actuated control does not win".
    sat = sc["saturated"]
    act = sat["controllers"]["actuated"]
    thr = pf["saturated"]["throughput_vph"]
    need(f"Delay is {pf['saturated']['mean_delay_s']['mean']:.1f} s/veh "
         f"({pf['saturated']['mean_delay_pct']:.1f} %) worse", "saturation delay")
    need(f"throughput {-thr['mean']:.0f} [{-thr['hi']:.0f}, {-thr['lo']:.0f}] veh/h", "saturation throughput")
    need(f"({act['max_outs_per_h']['mean']:.0f} max-outs\n  vs {act['gap_outs_per_h']['mean']:.0f} gap-outs",
         "saturation terminations")
    need(f"longer by {ci(sc['high']['paired']['actuated_minus_fixed']['queue_p95_veh'])} vehicles", "high p95 queue")
    need(f"again slightly longer\n  ({signed_ci(sc['medium']['paired']['actuated_minus_fixed']['queue_p95_veh'])} veh)",
         "medium p95 queue vs fixed-time")
    need(f"slightly longer at medium demand ({signed_ci(sc['medium']['paired']['actuated_minus_sumo']['queue_p95_veh'])} veh)",
         "medium p95 queue vs SUMO")
    return errors


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--readme", type=Path, default=ROOT / "README.md")
    ap.add_argument("--results", type=Path, default=ROOT / "results" / "results.json")
    args = ap.parse_args()
    readme = args.readme.read_text(encoding="utf-8").replace("\r\n", "\n")
    res = json.loads(args.results.read_text(encoding="utf-8"))
    errors = check(readme, res)
    for e in errors:
        print(f"MISMATCH {e}", file=sys.stderr)
    if errors:
        return 1
    print("README numbers match results/results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
