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
# Three tiers:
#   MODE=full (default) -- connectivity check + ${DURATION}s bot soak with
#     NAVSTAT stuck/lava thresholds. Slow, and stuck/lava readings get noisy
#     under memory pressure on constrained devices.
#   MODE=conn -- connectivity check only. Launches 1 bot (the first bot's
#     nav_ready call is what triggers the mesh build), watches the log, and
#     kills the server the moment the CONNECTIVITY summary prints (~20-30s
#     per map instead of ${DURATION}s+). Deterministic under load; use this
#     as the fast full-suite regression gate after navmesh changes.
#   MODE=bakesum -- conn checks PLUS exact bake-output comparison against
#     the checked-in baseline (src/tools/baselines/bakesum.txt). The bake
#     is deterministic, so a structural refactor that claims to preserve
#     behavior must produce a byte-identical BAKESUM line on every map.
#     Set UPDATE_BASELINE=1 to (re)capture the baseline after a reviewed,
#     intentional bake change.
#
set -uo pipefail

NQSERVER="${NQSERVER:-$HOME/bin/nqserver}"
GAMEDIR="${GAMEDIR:-$HOME/quake}"
GAME="${GAME:-ffa}"
MODE="${MODE:-full}"
BATCH_SIZE="${BATCH_SIZE:-6}"
OUTDIR="${OUTDIR:-$GAMEDIR/navruns/harness}"
if [[ "$MODE" == "conn" || "$MODE" == "bakesum" ]]; then
	BOTS=1
	DURATION="${DURATION:-90}" # worst-case mesh-build wait; early-kill makes the typical map much faster
elif [[ "$MODE" == "full" ]]; then
	BOTS="${BOTS:-4}"
	DURATION="${DURATION:-60}"
else
	echo "error: MODE must be 'full', 'conn', or 'bakesum', got '$MODE'" >&2
	exit 1
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAKESUM_BASELINE="${BAKESUM_BASELINE:-$SCRIPT_DIR/baselines/bakesum.txt}"
UPDATE_BASELINE="${UPDATE_BASELINE:-0}"
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

echo "Nav harness: mode=$MODE maps=[$MAPS] bots=$BOTS duration=${DURATION}s batch=$BATCH_SIZE"

# Matches only the final summary line (or the two skip variants), not the
# interim "N/M spawns resolve to a navmesh floor poly" line.
CONN_SUMMARY_RE="^Nav: CONNECTIVITY: ([0-9]+/[0-9]+ spawns unreachable|no spawn)"

# nqserver falls back to shareware mode (zero bot output) unless run
# from the directory holding id1/ and the game dir.
cd "$GAMEDIR" || exit 1

run_map() {
	local m="$1" pid
	timeout "$((DURATION + 20))" "$NQSERVER" -dedicated "$BOTS" -port 0 -game "$GAME" \
		+deathmatch 1 +skill 2 +temp1 "$BOTS" +map "$m" \
		>"$OUTDIR/$m.log" 2>&1 &
	pid=$!
	if [[ "$MODE" == "conn" || "$MODE" == "bakesum" ]]; then
		# The connectivity report is a one-shot at mesh-build time; once the
		# summary line lands there is nothing left to measure, so kill the
		# server instead of waiting out the clock.
		while kill -0 "$pid" 2>/dev/null; do
			grep -qE "$CONN_SUMMARY_RE" "$OUTDIR/$m.log" && break
			sleep 1
		done
		kill "$pid" 2>/dev/null
	fi
	wait "$pid" 2>/dev/null
}

n=0
for m in $MAPS; do
	run_map "$m" &
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
		# Two "^Nav: CONNECTIVITY:" lines are printed per map: an interim
		# "N/M spawns resolve to a navmesh floor poly" line, then the real
		# summary "N/M spawns unreachable, N/M items unreachable" line.
		# Must match the summary specifically -- grep -m1 on the bare
		# prefix used to grab the interim line instead, silently skipping
		# the unreachable-count check (e2m4/e3m5/e4m7 wrongly passed).
		conn_line="$(grep -m1 -E "$CONN_SUMMARY_RE" "$log" || true)"
		if [[ -z "$conn_line" ]]; then
			status="FAIL"
			reasons+=("no CONNECTIVITY report found")
		elif [[ "$conn_line" == *"no spawn points found"* ]]; then
			status="FAIL"
			reasons+=("no spawn points found on map")
		elif [[ "$conn_line" == *"no spawn resolves to a navmesh floor poly"* ]]; then
			status="FAIL"
			reasons+=("no spawn resolves to a navmesh floor poly")
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

		# Exact bake-output comparison. The BAKESUM line prints at
		# mesh-build time, before the CONNECTIVITY summary the
		# early-kill waits on, so it's always in the log by now.
		if [[ "$MODE" == "bakesum" ]]; then
			bake_line="$(grep -m1 "^Nav: BAKESUM map=$m " "$log" | sed 's/^Nav: //' || true)"
			if [[ -z "$bake_line" ]]; then
				status="FAIL"
				reasons+=("no BAKESUM line found")
			else
				echo "$bake_line" >>"$OUTDIR/bakesum.txt"
				if [[ "$UPDATE_BASELINE" != "1" ]]; then
					base_line="$(grep -m1 "^BAKESUM map=$m " "$BAKESUM_BASELINE" 2>/dev/null || true)"
					if [[ -z "$base_line" ]]; then
						status="FAIL"
						reasons+=("map missing from baseline $BAKESUM_BASELINE (run with UPDATE_BASELINE=1 to capture)")
					elif [[ "$bake_line" != "$base_line" ]]; then
						status="FAIL"
						reasons+=("bake output diverged: got [$bake_line] want [$base_line]")
					fi
				fi
			fi
		fi

		# Behavioral (NAVSTAT) thresholds only apply to the full-soak tier;
		# conn mode kills the server before any 10s NAVSTAT window elapses.
		if [[ "$MODE" == "full" ]]; then
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

if [[ "$MODE" == "bakesum" && "$UPDATE_BASELINE" == "1" && "$fail_count" -eq 0 ]]; then
	mkdir -p "$(dirname "$BAKESUM_BASELINE")"
	cp "$OUTDIR/bakesum.txt" "$BAKESUM_BASELINE"
	echo "baseline updated: $BAKESUM_BASELINE"
fi

[[ "$fail_count" -eq 0 ]]
