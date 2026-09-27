"""Measure SUMO's saturation flow on this network, so the Webster plan uses the capacity the
simulated drivers actually deliver rather than a textbook 1800 veh/h/lane.

Method: queue 60 through vehicles (lane 1, through only) and 40 turning-across vehicles
(lane 2) behind a red on the north approach, release them, and time their crossings of a loop
at the stop line. Saturation headway = median headway from the 5th vehicle onwards (the first
four carry the start-up loss). Crossing times are SUMO's interpolated loop entry times, so
they are not rounded to the 0.5 s step. Repeated over 10 seeds. Writes results/calibration.json.
"""
from __future__ import annotations

import json
import statistics

from common import GENERATED, RESULTS, STEP_S

import libsumo as traci  # in-process SUMO; same API as traci


def one_run(seed: int) -> dict[str, list[float]]:
    routes = GENERATED / "calibration.rou.xml"
    with open(routes, "w", encoding="utf-8") as f:
        f.write('<routes>\n  <route id="T" edges="n_in s_out"/>\n  <route id="R" edges="n_in w_out"/>\n')
        vehicles = [(0.5 * i, "T", 1) for i in range(60)] + [(0.5 * i, "R", 2) for i in range(40)]
        for k, (t, r, lane) in enumerate(sorted(vehicles)):
            f.write(f'  <vehicle id="{r}{k}" route="{r}" depart="{t:.1f}" departLane="{lane}" departSpeed="max"/>\n')
        f.write("</routes>\n")
    add = GENERATED / "calibration.add.xml"
    with open(add, "w", encoding="utf-8") as f:
        f.write("<additional>\n")
        for lane in (1, 2):
            f.write(f'  <inductionLoop id="stop{lane}" lane="n_in_{lane}" pos="-1" period="3600" file="NUL"/>\n')
        f.write("</additional>\n")
    traci.start(["sumo", "-n", str(GENERATED / "junction.net.xml"), "-r", str(routes), "-a", str(add),
                 "--step-length", str(STEP_S), "--seed", str(seed), "--no-step-log", "true",
                 "--no-warnings", "true", "--time-to-teleport", "-1"])
    links = json.loads((GENERATED / "link_groups.json").read_text())
    red = "r" * len(links)
    green = "".join("G" if g in ("N_T", "N_R") else "r" for g in links)
    crossings: dict[str, list[float]] = {"stop1": [], "stop2": []}
    seen: set[str] = set()
    while traci.simulation.getTime() < 400:
        t = traci.simulation.getTime()
        traci.trafficlight.setRedYellowGreenState("C", red if t < 120 else green)
        traci.simulationStep()
        for det in crossings:
            for vid, _length, entry, _exit, _type in traci.inductionloop.getVehicleData(det):
                if vid not in seen and entry >= 0:
                    seen.add(vid)
                    crossings[det].append(entry)
    traci.close()
    return crossings


def main() -> None:
    headways: dict[str, list[float]] = {"stop1": [], "stop2": []}
    for seed in range(1, 11):
        for det, times in one_run(seed).items():
            times.sort()
            headways[det] += [b - a for a, b in zip(times[4:], times[5:])]
    out = {}
    for det, key in (("stop1", "through_vphpl"), ("stop2", "turn_across_vphpl")):
        h = statistics.median(headways[det])
        out[key] = round(3600.0 / h)
        out[key.replace("vphpl", "median_headway_s")] = h
        out[key.replace("vphpl", "headways_measured")] = len(headways[det])
    out["method"] = "median discharge headway from the 5th queued vehicle onwards, 10 seeds, step 0.5 s"
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "calibration.json").write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
