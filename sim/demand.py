"""Seeded Poisson demand for each scenario, and the matching demand file for the Webster plan.

Every movement (approach x turn) gets independent Poisson arrivals at its hourly rate over
[0, DEMAND_END_S). Vehicle ids are "<approach><turn>.<n>", e.g. "NR.12", so the analysis can
report delay per movement.
"""
from __future__ import annotations

import json
import random

from common import (APPROACHES, DEMAND_END_S, DESTINATION, GENERATED, RESULTS, SCENARIOS, TURN_SHARE,
                    group_of)


def movement_rates(scenario: dict[str, float]) -> dict[tuple[str, str], float]:
    return {(a, turn): scenario[a] * share for a in APPROACHES for turn, share in TURN_SHARE.items()}


def write_routes(name: str, seed: int) -> str:
    rng = random.Random(f"{name}/{seed}")  # string seeds are stable across Python versions
    vehicles = []
    for (a, turn), vph in movement_rates(SCENARIOS[name]).items():
        t, k = 0.0, 0
        while True:
            t += rng.expovariate(vph / 3600.0)
            if t >= DEMAND_END_S:
                break
            vehicles.append((round(t, 1), f"{a}{turn}.{k}", f"{a}{turn}"))
            k += 1
    vehicles.sort()
    path = GENERATED / "routes" / f"{name}_{seed}.rou.xml"
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("<routes>\n")
        f.write('  <vType id="car" vClass="passenger" length="5" minGap="2.5" accel="2.6" decel="4.5" sigma="0.5"/>\n')
        for (a, turn), dest in DESTINATION.items():
            f.write(f'  <route id="{a}{turn}" edges="{a.lower()}_in {dest}"/>\n')
        for t, vid, route in vehicles:
            f.write(f'  <vehicle id="{vid}" type="car" route="{route}" depart="{t:.1f}" departLane="best" departSpeed="max"/>\n')
        f.write("</routes>\n")
    return str(path)


def write_demand_file(name: str) -> str:
    """Per-signal-group flows for tsc::webster_for, using the calibrated saturation flows."""
    cal = json.loads((RESULTS / "calibration.json").read_text(encoding="utf-8"))
    flows: dict[str, float] = {}
    for (a, turn), vph in movement_rates(SCENARIOS[name]).items():
        g = group_of(a, turn)
        flows[g] = flows.get(g, 0.0) + vph
    groups = {}
    for g, vph in flows.items():
        across = g.endswith("_R")
        groups[g] = {"flow_vph": vph, "lanes": 1 if across else 2,
                     # through group: lane 0 is shared with near-side turners, lane 1 is through only
                     "saturation_vphpl": cal["turn_across_vphpl"] if across else cal["through_group_vphpl"]}
    path = GENERATED / f"{name}.demand.json"
    path.write_text(json.dumps({"scenario": name, "groups": groups}, indent=2), encoding="utf-8")
    return str(path)


if __name__ == "__main__":
    for scenario in SCENARIOS:
        print(write_demand_file(scenario), write_routes(scenario, 1))
