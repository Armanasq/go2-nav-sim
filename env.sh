# Source this in every shell:  source env.sh
# ROS 2 Humble + this workspace, with a clean PATH (a venv on PATH breaks rosidl/empy during the build).
unset VIRTUAL_ENV; export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '/.venv/' | paste -sd:)
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$_ROOT/install/setup.bash" ] && source "$_ROOT/install/setup.bash"

export ROS_DOMAIN_ID=42                       # keeps this stack off any other ROS graph on the machine
export GAZEBO_MODEL_DATABASE_URI=""           # never fetch models online (hangs when the network is slow)

# Sensor plugins (camera, ray, imu, p3d) ship in ros-humble-gazebo-plugins. scripts/setup.sh unpacks them into overlay/ when they
# are not installed system-wide and sudo is unavailable.
if [ -d "$_ROOT/overlay/root/opt/ros/humble/lib" ]; then
  _OV="$_ROOT/overlay/root/opt/ros/humble/lib"
  export GAZEBO_PLUGIN_PATH="$_OV:${GAZEBO_PLUGIN_PATH}"
  export LD_LIBRARY_PATH="$_OV:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH}"
fi

# Render the cameras and the GPU lidar on the NVIDIA GPU when there is one (the default GL device may be an integrated GPU).
command -v nvidia-smi >/dev/null 2>&1 && export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia

# Cyclone DDS on loopback. With Fast DDS, service responses were dropped on a busy machine and Nav2's lifecycle bring-up timed out.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$_ROOT/config/cyclonedds.xml"
