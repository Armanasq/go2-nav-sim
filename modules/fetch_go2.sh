#!/bin/bash
# Fetch the Unitree Go2 MJCF model from MuJoCo Menagerie at a pinned commit into modules/assets/unitree_go2/ (not committed).
set -eo pipefail
cd "$(dirname "$0")"
SHA=79a68d43c43bf2a2b281495be4c0fe7759506db2
RAW=https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/$SHA/unitree_go2
DST=assets/unitree_go2; mkdir -p "$DST/assets"
for f in go2.xml scene.xml LICENSE README.md; do curl -fsSL --retry 3 -o "$DST/$f" "$RAW/$f"; done
for f in $(curl -fsSL "https://api.github.com/repos/google-deepmind/mujoco_menagerie/contents/unitree_go2/assets?ref=$SHA" | python3 -c "import json,sys; print(' '.join(x['name'] for x in json.load(sys.stdin)))"); do
  curl -fsSL --retry 3 -o "$DST/assets/$f" "$RAW/assets/$f"
done
echo "Go2 MJCF at $DST ($(ls $DST/assets | wc -l) mesh files)"
