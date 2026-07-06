#!/usr/bin/env bash
#
# 1v1 FFA decision-quality soak.
#
# Runs exactly two bots deathmatch on each map, capturing the engine's
# DAMAGE/PICKUP/GOAL_PICK telemetry lines (see nav_bot.cpp) for both bots.
# Unlike nav_harness.sh this has no pass/fail thresholds -- there's no
# ground truth for "good item choice" the way there is for reachability.
# Output is meant for src/tools/ffa_triage.py to turn into a human-readable
# per-bot timeline.
#
# Usage: src/tools/ffa_1v1_harness.sh [map ...]
#   (defaults to MAPS below if no maps given on the command line)
#
set -uo pipefail

NQSERVER="${NQSERVER:-$HOME/bin/nqserver}"
GAMEDIR="${GAMEDIR:-$HOME/quake}"
GAME="${GAME:-ffa}"
BOTS=2
DURATION="${DURATION:-300}"
OUTDIR="${OUTDIR:-$GAMEDIR/navruns/ffa}"

ID1_DM_MAPS="dm1 dm2 dm3 dm4 dm5 dm6"
MAPS="${*:-$ID1_DM_MAPS}"

if [[ ! -x "$NQSERVER" ]]; then
	echo "error: nqserver not found/executable at $NQSERVER" >&2
	exit 1
fi

rm -rf "$OUTDIR"
mkdir -p "$OUTDIR"

echo "FFA 1v1 harness: maps=[$MAPS] duration=${DURATION}s"

# nqserver falls back to shareware mode (zero bot output) unless run
# from the directory holding id1/ and the game dir.
cd "$GAMEDIR" || exit 1

for m in $MAPS; do
	timeout "$((DURATION + 20))" "$NQSERVER" -dedicated "$BOTS" -port 0 -game "$GAME" \
		+deathmatch 1 +skill 2 +temp1 "$BOTS" +map "$m" \
		>"$OUTDIR/$m.log" 2>&1
	dmg=$(grep -c "^DAMAGE " "$OUTDIR/$m.log")
	pkp=$(grep -c "^PICKUP " "$OUTDIR/$m.log")
	gp=$(grep -c "^GOAL_PICK " "$OUTDIR/$m.log")
	printf "%-6s DAMAGE=%-5s PICKUP=%-5s GOAL_PICK=%-5s\n" "$m" "$dmg" "$pkp" "$gp"
done

echo "---"
echo "Logs in $OUTDIR — run src/tools/ffa_triage.py to build per-bot timelines"
