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
#   MODE=behav -- behavioral differential tier. Soaks each map ${RUNS}
#     times with nav_debug on, aggregates NAVSTAT/PICKUP/LINKFAIL/GOALFAIL
#     into per-bot-sim-minute rates (median across runs to shed noise),
#     and compares against the checked-in baseline
#     (src/tools/baselines/behav.txt) with tolerance bands: stuck +5 abs,
#     pickups -20% rel, linkfail +50% rel, goalfail x3 (relative bands
#     carry a small absolute slack so a zero baseline isn't
#     unfailable-against). Catches behavior
#     regressions that connectivity alone shipped twice (e1m2). Set
#     UPDATE_BASELINE=1 to (re)capture after a reviewed behavior change.
#     Note: the engine never seeds rand(), so repeat runs decorrelate via
#     wall-clock frame-timing jitter, not RNG seed. Deliberate tradeoff
#     (2026-07): explicit srand would force an engine rebuild + baseline
#     recapture for marginal statistical gain.
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
	DURATION="${DURATION:-240}" # worst-case mesh-build wait; early-kill makes the typical map much faster.
	# 90 covered id1 alone, but several mission-pack maps (bigger, more off-mesh
	# links) blow past that under normal batch-parallel load; 240 is cheap
	# since early-kill means fast maps never wait it out, it only raises the
	# ceiling for the slow ones. True outliers get their own budget below.
elif [[ "$MODE" == "full" ]]; then
	BOTS="${BOTS:-4}"
	DURATION="${DURATION:-60}"
elif [[ "$MODE" == "behav" ]]; then
	BOTS="${BOTS:-4}"
	DURATION="${DURATION:-120}"
	RUNS="${RUNS:-3}"
else
	echo "error: MODE must be 'full', 'conn', 'bakesum', or 'behav', got '$MODE'" >&2
	exit 1
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAKESUM_BASELINE="${BAKESUM_BASELINE:-$SCRIPT_DIR/baselines/bakesum.txt}"
BEHAV_BASELINE="${BEHAV_BASELINE:-$SCRIPT_DIR/baselines/behav.txt}"
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

# Items the map author sealed away in deathmatch -- geometry-proven, not
# nav bugs.  id's pattern: an SP hazard/trap is papered over with a
# DM-only func_wall (skill spawnflags 1792 = never spawns in SP), and
# items placed for the SP route get stranded behind it.
#   e2m6: the flooded oubliette is the SP collapse-floor trap; in DM the
#   trap-floor doors are NOT_IN_DEATHMATCH and func_wall *69 seals the
#   pit, stranding two bottom item_health that lack the 2048 flag
#   (the armor beside them has it).  Flood-fill of world+*69 hull-0
#   confirms no entry.
#   e2m4: DM-only func_walls *59/*60 floor the slime moat over at
#   z257-271, turning the envirosuit/armorInv slime secret into a 48u
#   crawl space no 56u player can enter (probed headroom along the
#   whole approach).
#   r1m3: item_health authored at (-532 1696 -640) settles, via the SAME
#   256u droptofloor trace the real engine runs (cross-checked byte-for-
#   byte against pr_cmds.c PF_droptofloor), onto a knife-edge rim at
#   z=-824 -- the lip where a deep pit's walls begin their drop to the
#   true floor 248u further down at z=-1072.  TRIDUMP confirms the rim is
#   real solid geometry (not a phantom edge), but it's proven too narrow
#   for a player-sized box to stand on anywhere near the item without
#   clipping the shaft walls, so Recast correctly erodes it out of the
#   walkable navmesh -- no snap-tie-break bug (unlike hip3m4) and no
#   nearby poly to widen the query into.  Geometry-proven unreachable
#   pickup, not a nav bug.
declare -A KNOWN_UNREACHABLE_ITEMS=( [e2m6]=2 [e2m4]=2 [r1m3]=1 )

# Maps whose bake genuinely needs more than the fast-gate's default DURATION
# budget -- not a nav bug, just a big navmesh.  nav_mesh_disable_orphan_slivers
# recomputes disjoint-component analysis after every off-mesh-link pass, and
# that cost scales with poly count + component count, not map size per se.
#   r1m4: 12925 polys, up to 341 pre-cull orphan components -- bake measured
#   at 11m16s of CPU time (real time stretches further under device
#   contention: 36m34s observed on a loaded box). CONNECTIVITY confirmed
#   clean (0/12 spawns, 0/90 items unreachable) once it's given enough time
#   to finish. Bumping the global default DURATION to cover this would slow
#   the fast gate down for all 68 other maps just for one outlier, so give
#   it its own budget instead.
#   hip3m1: confirmed clean solo (0/57 items unreachable) but doesn't finish
#   within the 240s general MP budget under batch-parallel load.
#   hip2m3: confirmed clean solo (0/84 items unreachable, incl. a rocket-jump
#   grab reach) but likewise blows the 240s budget under batch-parallel load.
#   e3m5: id1, not mission-pack -- pre-existing note above already flagged
#   this map's bake as load-dependent (observed <140s cool, >230s thermally
#   throttled); confirmed clean (0/6 spawns, 0/97 items unreachable) but
#   needed 600-900s under this session's sustained device load.
declare -A MAP_DURATION_OVERRIDE=( [r1m4]=1400 [hip3m1]=900 [hip2m3]=900 [e3m5]=900 )

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
	local m="$1" log="${2:-$OUTDIR/$1.log}" pid
	local extra=()
	# LINKFAIL only prints under nav_debug; the behav tier counts it.
	# Behav's timeout is a hard guard only: the soak clock is event-driven
	# (starts when the bake-complete marker prints), because bake time is
	# wildly load-dependent (e3m5 observed <140s cool, >230s thermally
	# throttled) and a wall-clock budget lets slow bakes eat the soak.
	# noexit 1: an exit-touching bot dies and respawns instead of firing
	# changelevel, which drops every fake-client and leaves the rest of
	# the soak on an empty server (all non-DM maps have reachable exits).
	local slack=20
	[[ "$MODE" == "behav" ]] && { extra=(+nav_debug 1 +noexit 1); slack=600; }
	local dur="${MAP_DURATION_OVERRIDE[$m]:-$DURATION}"
	# -k: nqserver doesn't reliably die on the initial SIGTERM mid-bake (observed
	# 900s timeout -> 2194s actual wall time on r1m4), so force a SIGKILL if it's
	# still alive 60s after the term signal instead of trusting it to exit.
	timeout -k 60 "$((dur + slack))" "$NQSERVER" -dedicated "$BOTS" -port 0 -game "$GAME" \
		+deathmatch 1 +skill 2 +temp1 "$BOTS" "${extra[@]}" +map "$m" \
		>"$log" 2>&1 &
	pid=$!
	if [[ "$MODE" == "conn" || "$MODE" == "bakesum" ]]; then
		# The connectivity report is a one-shot at mesh-build time; once the
		# summary line lands there is nothing left to measure, so kill the
		# server instead of waiting out the clock.
		while kill -0 "$pid" 2>/dev/null; do
			grep -qE "$CONN_SUMMARY_RE" "$log" && break
			sleep 1
		done
		kill "$pid" 2>/dev/null
	elif [[ "$MODE" == "behav" ]]; then
		while kill -0 "$pid" 2>/dev/null; do
			grep -qE "$CONN_SUMMARY_RE" "$log" && break
			sleep 2
		done
		# The soak is pinned in SIM time (NAVSTAT windows), not wall time:
		# sim speed floats with host load (0.4x-2.4x wall observed), and
		# pickup rate is nonstationary (initial item burst, then
		# respawn-limited), so equal-sim soaks are the only samples
		# comparable across runs, devices, and load conditions.
		local tgt=$((BOTS * DURATION / 10))
		while kill -0 "$pid" 2>/dev/null; do
			[[ "$(grep -c '^NAVSTAT ' "$log")" -ge "$tgt" ]] && break
			sleep 2
		done
		kill "$pid" 2>/dev/null
	fi
	wait "$pid" 2>/dev/null
}

n=0
if [[ "$MODE" == "behav" ]]; then
	# Runs are the outer loop so a map's repeats spread across different
	# load conditions instead of sharing one batch's noise profile.
	for r in $(seq 1 "$RUNS"); do
		for m in $MAPS; do
			run_map "$m" "$OUTDIR/$m.r$r.log" &
			n=$((n + 1))
			if [[ "$n" -ge "$BATCH_SIZE" ]]; then
				wait
				n=0
			fi
		done
	done
else
	for m in $MAPS; do
		run_map "$m" &
		n=$((n + 1))
		if [[ "$n" -ge "$BATCH_SIZE" ]]; then
			wait
			n=0
		fi
	done
fi
wait

median() { printf '%s\n' "$@" | sort -n | sed -n "$((($# + 1) / 2))p"; }

if [[ "$MODE" == "behav" ]]; then
	fail_count=0
	for m in $MAPS; do
		status="PASS"
		reasons=()
		stks=(); pkms=(); lfms=(); gfms=()
		for r in $(seq 1 "$RUNS"); do
			log="$OUTDIR/$m.r$r.log"
			if [[ ! -s "$log" ]] || grep -qE "Sys_Error|Segmentation fault|shareware version" "$log"; then
				status="CRASH"
				reasons+=("run $r crashed or produced no output")
				continue
			fi
			tgt=$((BOTS * DURATION / 10))
			ns_have="$(grep -c '^NAVSTAT ' "$log")"
			if [[ "$ns_have" -lt "$tgt" ]]; then
				status="FAIL"
				reasons+=("run $r has $ns_have/$tgt sim windows (bake never finished or hard guard hit)")
				continue
			fi
			# Rates are per bot-minute of SIM time (one NAVSTAT line = 10
			# sim-seconds of one bot), and the awk exits at exactly the
			# window target so overshoot past the kill can't skew a
			# nonstationary rate. Wall time appears nowhere in the metrics.
			read -r stk pkm lfm gfm <<<"$(awk -v tgt="$tgt" '
				/^NAVSTAT / { ns++; if (match($0, / stk=[0-9.]+/)) { s += substr($0, RSTART + 5, RLENGTH - 5) }
					if (ns >= tgt) exit }
				/^PICKUP / { pk++ }
				/^LINKFAIL / { lf++ }
				/^GOALFAIL / { gf++ }
				END { bm = ns / 6; printf "%.1f %.2f %.2f %.2f", s / ns, pk / bm, lf / bm, gf / bm }' "$log")"
			stks+=("$stk"); pkms+=("$pkm"); lfms+=("$lfm"); gfms+=("$gfm")
		done

		if [[ "$status" == "PASS" ]]; then
			stk="$(median "${stks[@]}")"
			pkm="$(median "${pkms[@]}")"
			lfm="$(median "${lfms[@]}")"
			gfm="$(median "${gfms[@]}")"
			echo "BEHAV map=$m stk=$stk pkm=$pkm lfm=$lfm gfm=$gfm" >>"$OUTDIR/behav.txt"
			if [[ "$UPDATE_BASELINE" != "1" ]]; then
				base_line="$(grep -m1 "^BEHAV map=$m " "$BEHAV_BASELINE" 2>/dev/null || true)"
				if [[ -z "$base_line" ]]; then
					status="FAIL"
					reasons+=("map missing from baseline $BEHAV_BASELINE (run with UPDATE_BASELINE=1 to capture)")
				else
					read -r bstk bpkm blfm bgfm <<<"$(echo "$base_line" | \
						sed -n 's/^BEHAV map=[^ ]* stk=\([0-9.]*\) pkm=\([0-9.]*\) lfm=\([0-9.]*\) gfm=\([0-9.]*\)$/\1 \2 \3 \4/p')"
					if [[ -z "${bstk:-}" ]]; then
						status="FAIL"
						reasons+=("unparseable baseline line [$base_line]")
					else
						# Tolerance bands from the plan; each relative band gets a
						# small absolute slack so a zero baseline stays failable.
						awk -v a="$stk" -v b="$bstk" 'BEGIN{exit !(a > b + 5)}' && \
							{ status="FAIL"; reasons+=("stuck $stk% > baseline $bstk% + 5"); }
						awk -v a="$pkm" -v b="$bpkm" 'BEGIN{exit !(a < b * 0.8 - 0.5)}' && \
							{ status="FAIL"; reasons+=("pickups/min $pkm < baseline $bpkm - 20%"); }
						awk -v a="$lfm" -v b="$blfm" 'BEGIN{exit !(a > b * 1.5 + 0.5)}' && \
							{ status="FAIL"; reasons+=("linkfail/min $lfm > baseline $blfm + 50%"); }
						# Goal abandons are bursty: a 3.5x swing between runs with
						# an otherwise identical pickup rate was observed (e3m5),
						# so the band is deliberately wider than linkfail's.
						awk -v a="$gfm" -v b="$bgfm" 'BEGIN{exit !(a > b * 3 + 1)}' && \
							{ status="FAIL"; reasons+=("goalfail/min $gfm > baseline $bgfm x3"); }
					fi
				fi
			fi
		fi

		if [[ "$status" == "PASS" ]]; then
			printf "%-6s %-6s stk=%s pkm=%s lfm=%s gfm=%s\n" "$m" "$status" "$stk" "$pkm" "$lfm" "$gfm"
		else
			fail_count=$((fail_count + 1))
			printf "%-6s %-6s %s\n" "$m" "$status" "$(IFS='; '; echo "${reasons[*]}")"
		fi
	done

	echo "---"
	total=$(echo "$MAPS" | wc -w)
	echo "$((total - fail_count))/$total maps passed"
	if [[ "$UPDATE_BASELINE" == "1" && "$fail_count" -eq 0 ]]; then
		mkdir -p "$(dirname "$BEHAV_BASELINE")"
		cp "$OUTDIR/behav.txt" "$BEHAV_BASELINE"
		echo "baseline updated: $BEHAV_BASELINE"
	fi
	exit "$((fail_count > 0))"
fi

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
					known="${KNOWN_UNREACHABLE_ITEMS[$m]:-0}"
					item_eff=$((item_bad - known))
					[[ "$item_eff" -lt 0 ]] && item_eff=0
					item_pct=$((100 * item_eff / item_tot))
					if [[ "$item_pct" -gt "$MAX_ITEM_UNREACHABLE_PCT" ]]; then
						status="FAIL"
						reasons+=("$item_bad/$item_tot items unreachable (${item_pct}% over known $known)")
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
