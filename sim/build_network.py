"""Build the single four-arm junction with netconvert, plus the loop detectors and SUMO's own
actuated programme (the reference controller), all derived from configs/intersection.json.

Outputs (sim/generated/): junction.net.xml, detectors.add.xml, sumo_actuated.add.xml and
sumo_actuated_noskip.add.xml (the two variants of the reference controller),
link_groups.json (which signal group each SUMO link index belongs to).
"""
from __future__ import annotations

import json
import xml.etree.ElementTree as ET

from common import CONFIG, GENERATED, load_config, run

ARM_M = 500.0
SPEED = 13.89  # 50 km/h


def write_plain_network() -> None:
    nodes = {"C": (0, 0), "n": (0, ARM_M), "s": (0, -ARM_M), "e": (ARM_M, 0), "w": (-ARM_M, 0)}
    with open(GENERATED / "junction.nod.xml", "w", encoding="utf-8") as f:
        f.write("<nodes>\n")
        for nid, (x, y) in nodes.items():
            kind = "traffic_light" if nid == "C" else "priority"
            f.write(f'  <node id="{nid}" x="{x}" y="{y}" type="{kind}"/>\n')
        f.write("</nodes>\n")
    with open(GENERATED / "junction.edg.xml", "w", encoding="utf-8") as f:
        f.write("<edges>\n")
        for arm in "nsew":
            # Approaches: lane 0 near-side turn + through, lane 1 through, lane 2 turn across.
            f.write(f'  <edge id="{arm}_in" from="{arm}" to="C" numLanes="3" speed="{SPEED}"/>\n')
            f.write(f'  <edge id="{arm}_out" from="C" to="{arm}" numLanes="2" speed="{SPEED}"/>\n')
        f.write("</edges>\n")
    turns = {  # (from arm) -> near-side, through, across (left-hand traffic)
        "n": ("e", "s", "w"), "s": ("w", "n", "e"), "e": ("s", "w", "n"), "w": ("n", "e", "s"),
    }
    with open(GENERATED / "junction.con.xml", "w", encoding="utf-8") as f:
        f.write("<connections>\n")
        for arm, (near, through, across) in turns.items():
            f.write(f'  <connection from="{arm}_in" to="{near}_out" fromLane="0" toLane="0"/>\n')
            f.write(f'  <connection from="{arm}_in" to="{through}_out" fromLane="0" toLane="0"/>\n')
            f.write(f'  <connection from="{arm}_in" to="{through}_out" fromLane="1" toLane="1"/>\n')
            f.write(f'  <connection from="{arm}_in" to="{across}_out" fromLane="2" toLane="1"/>\n')
        f.write("</connections>\n")


def link_groups(cfg: dict, net_path) -> list[str]:
    """Signal group for every link index of the traffic light, from the net's connections."""
    movements = {}
    for group, pairs in cfg["sumo"]["movements"].items():
        for frm, to in pairs:
            movements[(frm, to)] = group
    tls = cfg["sumo"]["tls_id"]
    by_index: dict[int, str] = {}
    for c in ET.parse(net_path).getroot().iter("connection"):
        if c.get("tl") != tls:
            continue
        key = (c.get("from"), c.get("to"))
        if key not in movements:
            raise SystemExit(f"connection {key} is not assigned to a signal group in {CONFIG}")
        by_index[int(c.get("linkIndex"))] = movements[key]
    return [by_index[i] for i in range(len(by_index))]


def write_detectors(cfg: dict, net_path) -> None:
    lengths = {l.get("id"): float(l.get("length")) for l in ET.parse(net_path).getroot().iter("lane")}
    setback = cfg["sumo"]["detector_setback_m"]
    overrides = cfg["sumo"].get("detector_setback_overrides_m", {})
    with open(GENERATED / "detectors.add.xml", "w", encoding="utf-8") as f:
        f.write("<additional>\n")
        for det, lane in cfg["sumo"]["detector_lanes"].items():
            pos = lengths[lane] - overrides.get(det, setback)
            f.write(f'  <inductionLoop id="{det}" lane="{lane}" pos="{pos:.2f}" period="3600" file="NUL"/>\n')
        f.write("</additional>\n")


# SUMO programmes for the reference controller, one file per variant (same programID, so the
# harness loads whichever file it is given).
SUMO_PROGRAMS = {"sumo": "sumo_actuated.add.xml", "sumo_noskip": "sumo_actuated_noskip.add.xml"}


def write_sumo_actuated(cfg: dict, groups_per_link: list[str], skip: bool) -> str:
    """SUMO's built-in actuated controller (type="actuated", SUMO 1.18) set up like ours.

    Both variants use the same detectors, the same per-phase gap and the same min/max greens
    and clearances as configs/intersection.json:
      * gap control: SUMO allows one loop per lane; each approach lane is given our 30 m
        advance loop as its custom detector (<param key=LANE value=LOOP>), and that lane's
        max-gap is the passage time of the loop's phase (3.0 s through, 2.5 s turn-across);
      * skip (variant "sumo"): after each clearance a phase without recall is served only if it
        has a call, decided with SUMO's switching rules (finalTarget on the phase). A call is the
        same as ours: any loop of the phase, including the call-only stop-bar loops, detected a
        vehicle since that phase's green ended (z:LOOP < r:LINK + yellow), or is occupied now.
        SUMO's default choice between several `next` phases compares summed detector
        priorities instead, which skips a turn phase whenever the through phase has more active
        loops, and it has no stop-bar loops;
      * no skip (variant "sumo_noskip"): every phase is served in order every cycle, for at
        least its min green.
    """
    def state(green: set[str], letter: str) -> str:
        return "".join(letter if g in green else "r" for g in groups_per_link)

    phases = cfg["phases"]
    names = [p["name"] for p in phases]
    n = len(phases)
    dets = cfg["detectors"]
    lanes = cfg["sumo"]["detector_lanes"]

    def call_condition(i: int) -> str:
        p = phases[i]
        link = next(k for k, g in enumerate(groups_per_link) if g in p["groups"])
        yellow = p["yellow_s"]
        terms = [f"(z:{d['id']} < r:{link} + {yellow})" for d in dets if d["phase"] == p["name"]]
        return " or ".join(terms).replace("<", "&lt;")  # XML attribute

    rows = []
    for i, p in enumerate(phases):
        green = set(p["groups"])
        base = 3 * i
        target = f"call_{p['name']}" if skip and p.get("recall", "none") == "none" else None
        rows.append((state(green, "G"), p["min_green_s"], p["max_green_s"], None, p["name"], target))
        rows.append((state(green, "y"), p["yellow_s"], p["yellow_s"], None, p["name"] + " yellow", None))
        nxt = (i + 1) % n
        options = None
        if skip and phases[nxt].get("recall", "none") == "none":
            # serve the next phase if it has a call (its finalTarget), otherwise the one after it
            options = [3 * nxt, 3 * ((nxt + 1) % n)]
        rows.append((state(set(), "r"), p["all_red_s"], p["all_red_s"], options, p["name"] + " all-red", None))
    assert len(rows) == 3 * n and len(set(names)) == n

    setback = cfg["sumo"]["detector_setback_m"]
    path = GENERATED / SUMO_PROGRAMS["sumo" if skip else "sumo_noskip"]
    with open(path, "w", encoding="utf-8") as f:
        f.write("<additional>\n")
        f.write(f'  <tlLogic id="{cfg["sumo"]["tls_id"]}" type="actuated" programID="sumo_actuated" offset="0">\n')
        f.write(f'    <param key="detector-gap" value="{setback / SPEED:.2f}"/>\n')
        f.write('    <param key="file" value="NUL"/>\n')
        for d in dets:
            if d.get("mode", "extend") != "extend":
                continue  # call-only loops take no part in gap control
            lane = lanes[d["id"]]
            passage = phases[names.index(d["phase"])]["passage_s"]
            f.write(f'    <param key="{lane}" value="{d["id"]}"/>\n')
            f.write(f'    <param key="max-gap:{lane}" value="{passage}"/>\n')
        if skip:
            for i, p in enumerate(phases):
                if p.get("recall", "none") == "none":
                    f.write(f'    <condition id="call_{p["name"]}" value="{call_condition(i)}"/>\n')
        for st, mn, mx, options, name, target in rows:
            nxt = f' next="{" ".join(map(str, options))}"' if options else ""
            tgt = f' finalTarget="{target}"' if target else ""
            f.write(f'    <phase duration="{mn}" minDur="{mn}" maxDur="{mx}" state="{st}"{nxt}{tgt} name="{name}"/>\n')
        f.write("  </tlLogic>\n</additional>\n")
    return str(path)


def main() -> None:
    GENERATED.mkdir(parents=True, exist_ok=True)
    cfg = load_config()
    write_plain_network()
    net = GENERATED / "junction.net.xml"
    run(["netconvert", "--lefthand", "--no-turnarounds", "--no-internal-links", "false",
         "--node-files", GENERATED / "junction.nod.xml", "--edge-files", GENERATED / "junction.edg.xml",
         "--connection-files", GENERATED / "junction.con.xml", "--tls.default-type", "static",
         "--output-file", net, "--no-warnings"], capture_output=True)
    groups = link_groups(cfg, net)
    (GENERATED / "link_groups.json").write_text(json.dumps(groups, indent=1), encoding="utf-8")
    write_detectors(cfg, net)
    write_sumo_actuated(cfg, groups, skip=True)
    write_sumo_actuated(cfg, groups, skip=False)
    print(f"network: {net} ({len(groups)} signalised links)")


if __name__ == "__main__":
    main()
