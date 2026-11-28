#!/bin/bash
# Every setting, 3 scored A-to-B runs each:   projects/p12_tune-nav2/run_all.sh [runs_dir]     (default ~/p12_runs/all)
# Rounds are interleaved (round 1 of every setting, then round 2, ...) so a slow spell on the machine does not hit one setting
# only. A run whose stack does not come up is retried (up to 3x); a run that comes up and fails to reach B is a result and kept.
# The bags, scores (.json), first costmaps (.npz) and mission logs stay in runs_dir (bags about 35 MB each). results/ holds the
# reference runs and is left alone; P12_COPY_RESULTS=1 copies your scores and mission logs over them.
HERE=$(cd "$(dirname "$0")" && pwd); D=$(realpath -m "${1:-$HOME/p12_runs/all}"); mkdir -p "$D"
SETTINGS=${SETTINGS:-"base infl_r045 infl_r150 scale_1 scale_10 polygon smac_mppi"}     # SETTINGS=polygon_norot run_all.sh: one setting only
for k in 1 2 3; do
  for s in $SETTINGS; do
    [ -f "$D/${s}_$k.json" ] && continue                         # already scored: lets you resume after Ctrl-C
    for try in 1 2 3; do
      rm -rf "$D/${s}_$k"*
      "$HERE/run_ab.sh" "$s" "$D/${s}_$k" > "$D/${s}_$k.out" 2>&1
      [ -f "$D/${s}_$k.json" ] && break
      echo "$s run $k: stack did not come up (attempt $try): $(tail -1 "$D/${s}_$k.out")"
    done
    echo "$s run $k: $(grep -h RESULT "$D/${s}_$k.log" 2>/dev/null)"
  done
done
cd "$HERE/../.."; bash scripts/stop.sh
[ "${P12_COPY_RESULTS:-0}" = 1 ] || exit 0
cp "$D"/*.json "$D"/*.npz "$HERE/results/"; for f in "$D"/*_[123].log; do cp "$f" "$HERE/results/$(basename "$f" .log).mission.log"; done
