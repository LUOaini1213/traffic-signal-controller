#!/usr/bin/env python3
"""Run the README's command-line block verbatim and pin the Webster plans to results.json.

1. Takes the fenced bash block that follows "Command-line tool:" in README.md and runs each
   command from the repository root, with `build/debug/` replaced by --build-dir and `python3`
   by this interpreter. Every command must exit 0; the replay summary must report 0 audit
   violations, 0 guard refusals and no fail-safe.
2. Runs `tsc webster` for every scenario in results/results.json and requires the same cycle,
   clamping and green splits (exact) and the same optimum cycle (within 1e-6 s).

    python3 scripts/check_readme_cli.py --build-dir build/debug
"""
from __future__ import annotations

import argparse
import json
import math
import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def readme_cli_commands(readme: str) -> list[str]:
    lines = readme.replace("\r\n", "\n").split("\n")
    start = lines.index("Command-line tool:")
    i = start + 1
    while not lines[i].startswith("```"):
        i += 1
    cmds = []
    for line in lines[i + 1:]:
        if line.startswith("```"):
            return cmds
        line = line.split("#", 1)[0].strip()
        if line:
            cmds.append(line)
    raise SystemExit("README: unterminated command-line block")


def resolve(cmd: str, build_dir: Path) -> list[str]:
    argv = shlex.split(cmd)
    if argv[0] == "python3":
        argv[0] = sys.executable
    elif argv[0].startswith("build/debug/"):
        exe = build_dir / argv[0][len("build/debug/"):]
        if not exe.exists() and exe.with_suffix(".exe").exists():
            exe = exe.with_suffix(".exe")
        argv[0] = str(exe)
    else:
        raise SystemExit(f"unexpected command in README block: {cmd!r}")
    return argv


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--build-dir", type=Path, default=ROOT / "build" / "debug")
    ap.add_argument("--readme", type=Path, default=ROOT / "README.md")
    args = ap.parse_args()
    build_dir = args.build_dir.resolve()
    errors: list[str] = []

    cmds = readme_cli_commands(args.readme.read_text(encoding="utf-8"))
    kinds = [shlex.split(c)[1] if not c.startswith("python3") else "python" for c in cmds]
    for needed in ("validate", "replay", "webster"):
        if needed not in kinds:
            errors.append(f"README command-line block has no `tsc {needed}` command")
    for cmd in cmds:
        r = subprocess.run(resolve(cmd, build_dir), cwd=ROOT, capture_output=True, text=True, timeout=300)
        print(f"$ {cmd}  -> exit {r.returncode}")
        if r.returncode != 0:
            errors.append(f"`{cmd}` exited {r.returncode}: {r.stderr.strip()[-400:]}")
            continue
        if " replay " in f" {cmd} ":
            summary = json.loads(r.stderr)
            for key, want in (("audit_violations", 0), ("guard_refusals", 0), ("failsafe", False)):
                if summary.get(key) != want:
                    errors.append(f"`{cmd}`: {key} = {summary.get(key)!r}, expected {want!r}")

    results = json.loads((ROOT / "results" / "results.json").read_text(encoding="utf-8"))
    tsc = resolve("build/debug/tsc", build_dir)[0]
    for name, scen in results["scenarios"].items():
        demand = ROOT / "sim" / "generated" / f"{name}.demand.json"
        r = subprocess.run([tsc, "webster", str(ROOT / "configs" / "intersection.json"), str(demand)],
                           cwd=ROOT, capture_output=True, text=True, timeout=60)
        if r.returncode != 0:
            errors.append(f"webster {name} exited {r.returncode}: {r.stderr.strip()}")
            continue
        got, want = json.loads(r.stdout), scen["webster"]
        for key in ("cycle_s", "clamped", "green_s"):
            if got[key] != want[key]:
                errors.append(f"webster {name}: {key} = {got[key]!r}, results.json has {want[key]!r}")
        if not math.isclose(got["optimal_cycle_s"], want["optimal_cycle_s"], rel_tol=0, abs_tol=1e-6):
            errors.append(f"webster {name}: optimal_cycle_s = {got['optimal_cycle_s']}, "
                          f"results.json has {want['optimal_cycle_s']}")
        print(f"webster {name}: cycle {got['cycle_s']:g} s, optimum {got['optimal_cycle_s']:.2f} s")

    for e in errors:
        print(f"FAIL {e}", file=sys.stderr)
    if errors:
        return 1
    print("README command-line block runs and Webster plans match results/results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
