"""Short SUMO smoke run for CI: every controller for 15 simulated minutes, plus a stuck-loop run.

Fails (exit 1) if any run crashes, shows an invariant violation, has a guard refusal, or
moves no traffic. Uses the committed results/calibration.json, so it takes about a minute.
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

from common import CONFIG, GENERATED, HARNESS, RESULTS

HERE = Path(__file__).parent


def main() -> int:
    if not HARNESS.exists():
        print(f"{HARNESS} not found (build the 'release' preset first)", file=sys.stderr)
        return 1
    if not (RESULTS / "calibration.json").exists():
        print("results/calibration.json missing; run sim/calibrate.py", file=sys.stderr)
        return 1
    subprocess.run([sys.executable, HERE / "build_network.py"], check=True)
    import demand

    demand_file = demand.write_demand_file("medium")
    routes = demand.write_routes("medium", 1)
    failures = []
    cases = [("fixed", []), ("actuated", []), ("sumo", []),
             ("actuated", ["--fault", "N_2:stuck-off:60", "--fault", "E_1:stuck-on:60"])]
    with tempfile.TemporaryDirectory() as tmp:
        for i, (controller, extra) in enumerate(cases):
            summary = Path(tmp) / f"{i}.json"
            cmd = [HARNESS, "--config", CONFIG, "--net", GENERATED / "junction.net.xml", "--routes", routes,
                   "--detectors", GENERATED / "detectors.add.xml", "--program", GENERATED / "sumo_actuated.add.xml",
                   "--demand", demand_file, "--controller", controller, "--seed", "1",
                   "--warmup", "120", "--demand-end", "600", "--end", "900", "--summary", summary, *extra]
            subprocess.run([str(c) for c in cmd], check=True)
            r = json.loads(summary.read_text())
            problems = []
            if r["invariant_violations"]:
                problems.append(f"{r['invariant_violations']} invariant violations: {r['invariant_messages'][:3]}")
            if r.get("guard_refusals"):
                problems.append(f"{r['guard_refusals']} guard refusals")
            if r["throughput_vph"] <= 0:
                problems.append("no vehicles completed their trip")
            if r.get("runtime", {}).get("late_events"):
                problems.append("late detector events")
            if extra and len(r.get("faults_declared", [])) != 0:
                # stuck_off_s / stuck_on_s in the config are longer than this 900 s run, except stuck-on (300 s)
                kinds = {f["kind"] for f in r["faults_declared"]}
                if kinds != {"stuck-on"}:
                    problems.append(f"unexpected faults declared: {r['faults_declared']}")
            if extra and not r.get("faults_declared"):
                problems.append("stuck-on loop from 60 s was not declared faulty by 900 s")
            label = f"{controller} {' '.join(extra)}".strip()
            print(f"{'ok  ' if not problems else 'FAIL'} {label}: throughput {r['throughput_vph']:.0f} veh/h, "
                  f"p95 queue {r['queue_p95_worst_approach_veh']:.0f} veh, violations {r['invariant_violations']}")
            failures += [f"{label}: {p}" for p in problems]
    for f in failures:
        print("  " + f, file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
