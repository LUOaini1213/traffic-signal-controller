"""Shared paths, scenario definitions and helpers for the SUMO experiments (stdlib only)."""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs" / "intersection.json"
GENERATED = ROOT / "sim" / "generated"
RESULTS = ROOT / "results"
BUILD = Path(os.environ.get("TSC_BUILD_DIR", ROOT / "build" / "release"))
HARNESS = BUILD / "tsc_sumo"

STEP_S = 0.5          # simulation step = controller tick
WARMUP_S = 300        # statistics ignore vehicles that entered before this
DEMAND_END_S = 3600   # vehicles are generated in [0, DEMAND_END_S)
SIM_END_S = 7200      # hard stop; everything still queued at this point is reported as unfinished
SEEDS = list(range(1, 11))

APPROACHES = ["N", "S", "E", "W"]
# Left-hand traffic (as in Singapore): L = near-side turn, R = turn across oncoming traffic.
TURN_SHARE = {"L": 0.15, "T": 0.70, "R": 0.15}

# Vehicles per hour arriving on each approach. "saturated" is beyond what the configured max
# greens can serve; it is there to show where actuated control stops helping.
SCENARIOS = {
    "low": {"N": 300, "S": 300, "E": 300, "W": 300},
    "medium": {"N": 600, "S": 600, "E": 600, "W": 600},
    "high": {"N": 1200, "S": 1200, "E": 1200, "W": 1200},
    "asymmetric": {"N": 1100, "S": 1100, "E": 300, "W": 300},
    "saturated": {"N": 1600, "S": 1600, "E": 1600, "W": 1600},
}

# Destination edge for each approach and turn (left-hand traffic).
DESTINATION = {
    ("N", "L"): "e_out", ("N", "T"): "s_out", ("N", "R"): "w_out",
    ("S", "L"): "w_out", ("S", "T"): "n_out", ("S", "R"): "e_out",
    ("E", "L"): "s_out", ("E", "T"): "w_out", ("E", "R"): "n_out",
    ("W", "L"): "n_out", ("W", "T"): "e_out", ("W", "R"): "s_out",
}


def load_config() -> dict:
    return json.loads(CONFIG.read_text(encoding="utf-8"))


def group_of(approach: str, turn: str) -> str:
    """Signal group that serves a movement: near-side turns share the through group."""
    return f"{approach}_{'R' if turn == 'R' else 'T'}"


def sumo_home() -> str:
    for candidate in (os.environ.get("SUMO_HOME"), "/usr/share/sumo"):
        if candidate and Path(candidate).exists():
            return candidate
    raise SystemExit("SUMO_HOME not set and /usr/share/sumo not found; install sumo (apt install sumo sumo-tools)")


def run(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    return subprocess.run([str(c) for c in cmd], check=True, **kw)
