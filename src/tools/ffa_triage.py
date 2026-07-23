#!/usr/bin/env python3
"""Reconstruct per-bot decision timelines from a ffa_1v1_harness.sh run.

There's no ground truth for "good item choice" the way there is for navmesh
reachability, so this doesn't pass/fail anything -- it merges the engine's
DAMAGE/PICKUP/GOAL_PICK telemetry lines (see nav_bot.cpp) into one
chronological timeline per bot per map, for a human to read and judge things
like "did it detour to health after combat" or "did it skip a bigger prize
for something cheaper". It does flag two heuristics inline as a reading aid,
not as a verdict:

  SKIPPED   - a GOAL_PICK where a reachable item with higher item_mag existed
              and wasn't picked
  SLOW HEAL - health dropped below LOW_HEALTH after a DAMAGE event and the
              bot didn't pick up health/armor for more than SLOW_HEAL_SECS

Usage: src/tools/ffa_triage.py [logdir]   (default ~/quake/navruns/ffa)
"""

import os
import re
import sys
from pathlib import Path

LOGDIR = Path(sys.argv[1] if len(sys.argv) > 1 else Path.home() / "quake" / "navruns" / "ffa")

LOW_HEALTH = float(os.environ.get("LOW_HEALTH", "40"))
SLOW_HEAL_SECS = float(os.environ.get("SLOW_HEAL_SECS", "15"))

# must match RES_* in src/frikbot/bot_goal.qc
RES_NAMES = {1: "HEALTH", 2: "ARMOR", 3: "WEAPON", 4: "AMMO", 5: "POWERUP"}

DAMAGE_RE = re.compile(
    r"^DAMAGE time=([\d.]+) target=(\S+) attacker=(\S+) amount=(-?[\d.]+) "
    r"health=(-?[\d.]+) armor=(-?[\d.]+)")
PICKUP_RE = re.compile(
    r"^PICKUP time=([\d.]+) bot=(\S+) item=(\S+) res=([\d.]+) mag=([\d.]+) "
    r"health=([\d.]+) armor=([\d.]+)")
GOAL_RE = re.compile(
    r"^GOAL_PICK time=([\d.]+) bot=(\S+) item=(\S+) want=([\d.]+) cost=(-?[\d.]+) "
    r"skip_item=(\S*) skip_mag=([\d.]+)")


def parse_log(log):
    """Return {bot_netname: [(time, kind, text, is_low_health_res)]}."""
    timelines = {}

    def add(bot, t, kind, text):
        timelines.setdefault(bot, []).append((t, kind, text))

    for line in log.read_text(errors="replace").splitlines():
        m = DAMAGE_RE.match(line)
        if m:
            t, target, attacker, amount, health, armor = m.groups()
            t = float(t)
            add(target, t, "DAMAGE",
                f"took {float(amount):.0f} dmg from {attacker} (health -> {float(health):.0f})")
            continue

        m = PICKUP_RE.match(line)
        if m:
            t, bot, item, res, mag, health, armor = m.groups()
            t = float(t)
            res_name = RES_NAMES.get(int(float(res)), "?")
            add(bot, t, "PICKUP",
                f"picked up {item} [{res_name} mag={float(mag):.2f}] "
                f"(health={float(health):.0f} armor={float(armor):.0f})")
            continue

        m = GOAL_RE.match(line)
        if m:
            t, bot, item, want, cost, skip_item, skip_mag = m.groups()
            t = float(t)
            text = f"goal -> {item} (want={float(want):.2f} cost={float(cost):.0f})"
            if skip_item:
                text += f"  ** SKIPPED {skip_item} mag={float(skip_mag):.2f} (reachable) **"
            add(bot, t, "GOAL_PICK", text)
            continue

    return timelines


def annotate_slow_heal(events):
    """events: list of (time, kind, text) for one bot, already sorted.
    Returns the same list with SLOW HEAL markers appended where they apply."""
    out = []
    pending_since = None  # time health last dropped below LOW_HEALTH
    for t, kind, text in events:
        if kind == "DAMAGE":
            m = re.search(r"health -> (-?[\d.]+)", text)
            if m and float(m.group(1)) <= 0:
                # died: respawn resets health, stop expecting a heal
                pending_since = None
            elif m and float(m.group(1)) < LOW_HEALTH and pending_since is None:
                pending_since = t
        elif kind == "PICKUP" and pending_since is not None:
            if "HEALTH" in text or "ARMOR" in text:
                elapsed = t - pending_since
                if elapsed > SLOW_HEAL_SECS:
                    text += f"  ** SLOW HEAL: {elapsed:.0f}s below {LOW_HEALTH:.0f} hp **"
                pending_since = None
        out.append((t, kind, text))
    if pending_since is not None:
        out.append((events[-1][0], "NOTE",
                     f"never healed after dropping below {LOW_HEALTH:.0f} hp at t={pending_since:.1f}s"))
    return out


skip_count = 0
slow_heal_count = 0
for log in sorted(LOGDIR.glob("*.log")):
    mapname = log.stem
    timelines = parse_log(log)
    if not timelines:
        continue
    print(f"=== {mapname} ===")
    for bot in sorted(timelines):
        events = sorted(timelines[bot], key=lambda e: e[0])
        events = annotate_slow_heal(events)
        print(f"-- {bot} --")
        for t, kind, text in events:
            print(f"  [{t:7.1f}s] {kind:9s} {text}")
            if "SKIPPED" in text:
                skip_count += 1
            if "SLOW HEAL" in text or kind == "NOTE":
                slow_heal_count += 1

print("---")
print(f"{skip_count:4d} SKIPPED (goal picked over a higher-mag reachable item)")
print(f"{slow_heal_count:4d} SLOW HEAL / never-healed occurrences")
