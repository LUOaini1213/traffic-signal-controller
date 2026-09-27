"""Run every simulation the README reports and store one JSON record per run.

  python3 sim/run_experiments.py              # all scenarios x 10 seeds x 4 controllers + fault/ablation runs
  python3 sim/run_experiments.py --seeds 2 --scenarios medium   # quicker subset

Requires the Release build with the SUMO harness (cmake --preset release; TSC_BUILD_DIR points
at it). Per-run records go to results/runs/*.json (git-ignored); sim/analyze.py turns them into
results/results.json and results/results.md.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from build_network import SUMO_PROGRAMS
from common import (CONFIG, DEMAND_END_S, GENERATED, HARNESS, RESULTS, SCENARIOS, SEEDS, SIM_END_S,
                    WARMUP_S, group_of)

RUNS = RESULTS / "runs"
# "sumo" and "sumo_noskip" are SUMO's own actuated controller in two set-ups (see
# build_network.write_sumo_actuated); the harness runs both as --controller sumo.
CONTROLLERS = ["fixed", "actuated", "sumo", "sumo_noskip"]

# Detector-fault experiment: all four loops of the north-south turn-across phase (advance
# loops N_2, S_2 and stop-bar loops N_2s, S_2s) stick "off" at 600 s under medium demand, as if
# the detector card serving them failed. That phase (NS_right) has no recall, so without fault
# handling it is never called again. With fault handling the loops are declared stuck-off once
# they have been silent for faults.stuck_off_s (900 s) while the other loops saw traffic, i.e.
# at about 1500 s, and the phase falls back to max recall.
FAULT_SCENARIO = "medium"
FAULT_ARGS = [a for d in ("N_2", "S_2", "N_2s", "S_2s") for a in ("--fault", f"{d}:stuck-off:600")]
FAULT_VARIANTS = {
    "no_fault": [],
    "fault_handling_off": [*FAULT_ARGS, "--no-fault-handling"],
    "fault_handling_on": FAULT_ARGS,
}


def trip_stats(tripinfo: Path, planned_departs: dict[str, float], sim_end_s: float) -> dict:
    """Delay = time lost to the signal and queues (SUMO timeLoss) + time waiting to enter the network.

    Vehicles still driving or queued when the run is cut off at sim_end_s only carry the delay
    accumulated so far, and vehicles that were never inserted (their entry lane stayed blocked)
    are not in the tripinfo at all; they are counted with delay = sim_end_s - planned departure.
    When either kind occurs, the mean is a lower bound (flagged by "delay_is_lower_bound").
    """
    per_movement: dict[str, list[float]] = {}
    delays: list[float] = []
    unfinished = 0
    seen: set[str] = set()

    def add(vid: str, planned: float, d: float) -> None:
        if not WARMUP_S <= planned < DEMAND_END_S:
            return
        delays.append(d)
        per_movement.setdefault(vid.split(".")[0], []).append(d)

    for _, trip in ET.iterparse(tripinfo):  # streaming: tripinfo files get large
        if trip.tag != "tripinfo":
            continue
        a = dict(trip.attrib)
        trip.clear()
        seen.add(a["id"])
        planned = float(a["depart"]) - float(a["departDelay"])
        if float(a.get("arrival", "-1")) < 0 and WARMUP_S <= planned < DEMAND_END_S:
            unfinished += 1
        add(a["id"], planned, float(a["timeLoss"]) + float(a["departDelay"]))
    never = {vid: t for vid, t in planned_departs.items() if vid not in seen}
    never_in_window = 0
    for vid, planned in never.items():
        never_in_window += WARMUP_S <= planned < DEMAND_END_S
        add(vid, planned, sim_end_s - planned)
    mean = lambda xs: sum(xs) / len(xs) if xs else None  # noqa: E731
    return {
        "mean_delay_s": mean(delays),
        "delay_is_lower_bound": unfinished + never_in_window > 0,
        "vehicles_measured": len(delays),
        "vehicles_not_finished_in_window": unfinished,
        "vehicles_never_inserted": len(never),
        "vehicles_never_inserted_in_window": never_in_window,
        "mean_delay_by_movement_s": {m: mean(v) for m, v in sorted(per_movement.items())},
        "mean_delay_by_group_s": {
            g: mean([d for m, v in per_movement.items() if group_of(m[0], m[1]) == g for d in v])
            for g in sorted({group_of(m[0], m[1]) for m in per_movement})
        },
    }


def run_one(job: dict) -> dict:
    out = RUNS / job["name"]
    if out.with_suffix(".json").exists():  # finished in an earlier, interrupted invocation
        return json.loads(out.with_suffix(".json").read_text(encoding="utf-8"))
    out.parent.mkdir(parents=True, exist_ok=True)
    summary, tripinfo = out.with_suffix(".summary.json"), out.with_suffix(".tripinfo.xml")
    routes = GENERATED / "routes" / f"{job['scenario']}_{job['seed']}.rou.xml"
    cmd = [str(HARNESS), "--config", str(job.get("config", CONFIG)), "--net", str(GENERATED / "junction.net.xml"),
           "--routes", str(routes), "--detectors", str(GENERATED / "detectors.add.xml"),
           "--program", str(GENERATED / SUMO_PROGRAMS.get(job["controller"], SUMO_PROGRAMS["sumo"])),
           "--demand", str(GENERATED / f"{job['scenario']}.demand.json"),
           "--controller", "sumo" if job["controller"].startswith("sumo") else job["controller"],
           "--seed", str(job["seed"]),
           "--warmup", str(WARMUP_S), "--demand-end", str(DEMAND_END_S), "--end", str(SIM_END_S),
           "--tripinfo", str(tripinfo), "--summary", str(summary), *job.get("extra", [])]
    subprocess.run(cmd, check=True, capture_output=True)
    rec = json.loads(summary.read_text(encoding="utf-8"))
    departs = {v.get("id"): float(v.get("depart")) for v in ET.parse(routes).getroot().iter("vehicle")}
    rec.update(trip_stats(tripinfo, departs, rec["sim_end_s"]))
    rec.update({k: job[k] for k in ("scenario", "seed", "experiment", "controller")})
    rec["variant"] = job.get("variant")
    tripinfo.unlink()  # large; everything needed is in the record
    summary.unlink()
    out.with_suffix(".json").write_text(json.dumps(rec, indent=1), encoding="utf-8")
    return rec


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", nargs="*", default=list(SCENARIOS))
    ap.add_argument("--seeds", type=int, default=len(SEEDS))
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--skip-fault", action="store_true")
    ap.add_argument("--skip-ablation", action="store_true")
    args = ap.parse_args()
    if not HARNESS.exists():
        sys.exit(f"{HARNESS} not found: build with `cmake --preset release && cmake --build build/release` "
                 "or set TSC_BUILD_DIR")

    here = Path(__file__).parent
    subprocess.run([sys.executable, here / "build_network.py"], check=True)
    if not (RESULTS / "calibration.json").exists():
        subprocess.run([sys.executable, here / "calibrate.py"], check=True)
    import demand  # after the network exists

    seeds = SEEDS[: args.seeds]
    jobs = []
    for scenario in args.scenarios:
        demand.write_demand_file(scenario)
        for seed in seeds:
            demand.write_routes(scenario, seed)
            for c in CONTROLLERS:
                jobs.append({"name": f"compare/{scenario}/{c}_{seed}", "experiment": "compare",
                             "scenario": scenario, "seed": seed, "controller": c})
    if not args.skip_ablation:
        # Ablation: the same controller with the stop-bar loops set to extend the green
        # instead of only placing calls.
        cfg = json.loads(CONFIG.read_text(encoding="utf-8"))
        for d in cfg["detectors"]:
            if d.get("mode") == "call":
                d["mode"] = "extend"
        alt = GENERATED / "intersection_stopbar_extend.json"
        alt.write_text(json.dumps(cfg, indent=1), encoding="utf-8")
        for scenario in args.scenarios:
            for seed in seeds:
                jobs.append({"name": f"ablation/{scenario}/stopbar_extend_{seed}", "experiment": "ablation",
                             "variant": "stopbar_extend", "scenario": scenario, "seed": seed,
                             "controller": "actuated", "config": str(alt)})
    if not args.skip_fault:
        demand.write_demand_file(FAULT_SCENARIO)
        for seed in seeds:
            demand.write_routes(FAULT_SCENARIO, seed)
            for variant, extra in FAULT_VARIANTS.items():
                jobs.append({"name": f"fault/{variant}_{seed}", "experiment": "fault", "variant": variant,
                             "scenario": FAULT_SCENARIO, "seed": seed, "controller": "actuated", "extra": extra})
    print(f"{len(jobs)} runs on {args.jobs} processes", flush=True)
    with mp.Pool(args.jobs) as pool:
        for i, rec in enumerate(pool.imap_unordered(run_one, jobs), 1):
            if i % 10 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)} done", flush=True)


if __name__ == "__main__":
    main()
