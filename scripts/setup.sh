#!/bin/bash
# One-time setup from a fresh clone:  scripts/setup.sh  [--fetch-only]
#   1. fetch the upstream Go2/CHAMP description at a pinned commit and apply patches/upstream.patch (--fetch-only stops here; CI uses it)
#   2. install the Gazebo sensor plugins into overlay/ if they are missing and sudo is not available
#   3. build the workspace
set -eo pipefail
cd "$(dirname "$0")/.."
UP=https://github.com/anujjain-dev/unitree-go2-ros2.git; SHA=fdac259518930c8f03e79189c4d0fdd00fc47483; DST=src/third_party/unitree-go2-ros2

if [ ! -d "$DST/.git" ]; then
  mkdir -p src/third_party && git init -q "$DST" && git -C "$DST" remote add origin "$UP"
  git -C "$DST" fetch -q --depth 1 origin "$SHA" && git -C "$DST" checkout -q FETCH_HEAD
fi
git -C "$DST" apply --check ../../../patches/upstream.patch 2>/dev/null && git -C "$DST" apply ../../../patches/upstream.patch && echo "patch applied" || echo "patch already applied"
[ "$1" = --fetch-only ] && exit 0

if ! ls /opt/ros/humble/lib/libgazebo_ros_camera.so >/dev/null 2>&1; then
  mkdir -p overlay/debs && cd overlay/debs
  for p in ros-humble-gazebo-plugins ros-humble-camera-info-manager ros-humble-camera-calibration-parsers; do apt-get -o Acquire::ForceIPv4=true download "$p"; done
  for d in *.deb; do dpkg -x "$d" ../root; done; cd ../..; touch overlay/COLCON_IGNORE; echo "plugins unpacked into overlay/"
fi

source ./env.sh
colcon build --symlink-install
echo "done. next:  source env.sh && scripts/up.sh"
