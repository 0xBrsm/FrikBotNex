#!/usr/bin/env bash
#
# Oracle-free navigation quality harness.
#
# Nav_Validate (nav_val.cpp) only has hand-authored ground truth for
# dm1-dm6. Every other map -- which is the whole point of automatic
# navmesh generation -- has none. This harness instead runs bots on a
# map list, reads the engine's own machine-parseable report lines
# (CONNECTIVITY at mesh-build time, NAVSTAT every 10s per bot), and
# applies pass/fail thresholds. Works on any map, no waypoints needed.
#
# Usage: src/tools/nav_harness.sh [map ...]
#   (defaults to MAPS below if no maps given on the command line)
#
set -uo pipefail

NQSERVER="${NQSERVER:-$HOME/bin/nqserver}"
GAMEDIR="${GAMEDIR:-$HOME/quake}"
GAME="${GAME:-ffa}"
BOTS="${BOTS:-4}"
DURATION="${DURATION:-60}"
BATCH_SIZE="${BATCH_SIZE:-6}"
OUTDIR="${OUTDIR:-$GAMEDIR/navruns/harness}"
ID1_MAPS="start \
	e1m1 e1m2 e1m3 e1m4 e1m5 e1m6 e1m7 \
	e2m1 e2m2 e2m3 e2m4 e2m5 e2m6 e2m7 \
	e3m1 e3m2 e3m3 e3m4 e3m5 e3m6 e3m7 \
	e4m1 e4m2 e4m3 e4m4 e4m5 e4m6 e4m7 e4m8 \
	end \
	dm1 dm2 dm3 dm4 dm5 dm6"
MAPS="${*:-$ID1_MAPS}"

# A map fails if any of these are crossed.
MAX_STUCK_PCT="${MAX_STUCK_PCT:-15}"
MAX_LAVA_PCT="${MAX_LAVA_PCT:-2}"
MAX_SPAWN_UNREACHABLE="${MAX_SPAWN_UNREACHABLE:-0}"
MAX_ITEM_UNREACHABLE_PCT="${MAX_ITEM_UNREACHABLE_PCT:-0}"

if [[ ! -x "$NQSERVER" ]]; then
	echo "error: nqserver not found/executable at $NQSERVER" >&2
	exit 1
fi

rm -rf "$OUTDIR"
mkdir -p "$OUTDIR"

echo "Nav harness: maps=[$MAPS] bots=$BOTS duration=${DURATION}s batch=$BATCH_SIZE"

# nqserver falls back to shareware mode (zero bot output) unless run
# from the directory holding id1/ and the game dir.
cd "$GAMEDIR" || exit 1

n=0
for m in $MAPS; do
	timeout "$((DURATION + 20))" "$NQSERVER" -dedicated "$BOTS" -port 0 -game "$GAME" \
		+deathmatch 1 +skill 2 +temp1 "$BOTS" +map "$m" \
		>"$OUTDIR/$m.log" 2>&1 &
	n=$((n + 1))
	if [[ "$n" -ge "$BATCH_SIZE" ]]; then
		wait
		n=0
	fi
done
wait

fail_count=0
for m in $MAPS; do
	log="$OUTDIR/$m.log"
	status="PASS"
	reasons=()

	if [[ ! -s "$log" ]] || grep -qE "Sys_Error|Segmentation fault|shareware version" "$log"; then
		status="CRASH"
		reasons+=("engine crashed or produced no output")
	fi

	if [[ "$status" != "CRASH" ]]; then
		conn_line="$(grep -m1 "^Nav: CONNECTIVITY:" "$log" || true)"
		if [[ -z "$conn_line" ]]; then
			status="FAIL"
			reasons+=("no CONNECTIVITY report found")
		elif [[ "$conn_line" == *"no spawn points found"* ]]; then
			status="FAIL"
			reasons+=("no spawn points found on map")
		else
			read -r spawn_bad spawn_tot item_bad item_tot <<<"$(echo "$conn_line" | \
				sed -n 's#.*: \([0-9]*\)/\([0-9]*\) spawns unreachable, \([0-9]*\)/\([0-9]*\) items unreachable#\1 \2 \3 \4#p')"
			if [[ -n "${spawn_bad:-}" ]]; then
				if [[ "$spawn_bad" -gt "$MAX_SPAWN_UNREACHABLE" ]]; then
					status="FAIL"
					reasons+=("$spawn_bad/$spawn_tot spawns unreachable")
				fi
				if [[ "${item_tot:-0}" -gt 0 ]]; then
					item_pct=$((100 * item_bad / item_tot))
					if [[ "$item_pct" -gt "$MAX_ITEM_UNREACHABLE_PCT" ]]; then
						status="FAIL"
						reasons+=("$item_bad/$item_tot items unreachable (${item_pct}%)")
					fi
				fi
			fi
		fi

		navstats="$(grep "^NAVSTAT " "$log" || true)"
		if [[ -z "$navstats" ]]; then
			status="FAIL"
			reasons+=("no NAVSTAT reports found (bots never ran)")
		else
			avg_stk="$(echo "$navstats" | sed -n 's/.* stk=\([0-9.]*\).*/\1/p' | \
				awk '{s+=$1; n++} END {if (n>0) printf "%.0f", s/n; else print 0}')"
			avg_lava="$(echo "$navstats" | sed -n 's/.* lava=\([0-9.]*\).*/\1/p' | \
				awk '{s+=$1; n++} END {if (n>0) printf "%.0f", s/n; else print 0}')"
			if [[ "$avg_stk" -gt "$MAX_STUCK_PCT" ]]; then
				status="FAIL"
				reasons+=("avg stuck ${avg_stk}% > ${MAX_STUCK_PCT}%")
			fi
			if [[ "$avg_lava" -gt "$MAX_LAVA_PCT" ]]; then
				status="FAIL"
				reasons+=("avg lava ${avg_lava}% > ${MAX_LAVA_PCT}%")
			fi
		fi
	fi

	if [[ "$status" == "PASS" ]]; then
		printf "%-6s %-6s\n" "$m" "$status"
	else
		fail_count=$((fail_count + 1))
		printf "%-6s %-6s %s\n" "$m" "$status" "$(IFS='; '; echo "${reasons[*]}")"
	fi
done

echo "---"
total=$(echo "$MAPS" | wc -w)
echo "$((total - fail_count))/$total maps passed"
[[ "$fail_count" -eq 0 ]]
