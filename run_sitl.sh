#!/bin/bash
# Launch gz-sim for the Hirrus flying wing. Run ArduPilot SITL in a second
# terminal (printed below) -- same 2-terminal workflow as ardupilot_gazebo's
# own Zephyr example (see ~/ardupilot_gazebo/README.md).
set -euo pipefail

HIRRUS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Always source ROS2's setup.bash (when present), not just when `gz` is
# missing from PATH: /usr/bin/gz also exists here (the unrelated gz-tools2
# apt package), so a "only if not found" check finds THAT one, silently
# skips sourcing, and gz-tools2's own GZ_CONFIG_PATH doesn't know about the
# gz-sim8/Harmonic plugins this project actually needs -- it fails with
# "I cannot find any available 'gz' command" despite a `gz` technically
# being on PATH. ROS2 Jazzy + Gazebo Harmonic is already this whole
# project's prerequisite (see AeroCanyon_Guard's own README), so there's no
# real "don't have ROS" case to preserve here.
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # ROS2's setup.bash isn't nounset-safe (references unset vars internally)
  # -- relax -u just for this line rather than dropping it script-wide.
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
if ! command -v gz >/dev/null 2>&1; then
  echo "error: 'gz' not found on PATH, and /opt/ros/jazzy/setup.bash didn't provide it either." >&2
  echo "Source whatever gives you the gz CLI (ROS2's setup.bash, or your Gazebo install) first." >&2
  exit 1
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$HIRRUS_DIR/models:$HIRRUS_DIR/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"

echo "In another terminal, run:"
echo "  sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HIRRUS_DIR/hirrus.parm --map --console"
echo
# `gz launch` isn't a registered subcommand on this system's (ROS-vendored)
# `gz` CLI -- `gz --commands` lists no "launch" at all, even with
# gz-launch7-cli installed. The actual binary (same one AeroCanyon_Guard's
# own README calls directly, same reason) still works fine called directly,
# positionally (no -f -- that flag doesn't exist on this binary, confirmed
# via its own --help).
GZ_LAUNCH_BIN=$(echo /usr/lib/*/gz/launch*/gz-launch 2>/dev/null | awk '{print $1}')

echo "For the browser view + manual control (optional, 2 more terminals -- each"
echo "needs 'gz' on PATH too, e.g. 'source /opt/ros/jazzy/setup.bash' first):"
echo "  cd $HIRRUS_DIR/web_viewer && ${GZ_LAUNCH_BIN:-gz-launch} websocket.gzlaunch"
echo "  cd $HIRRUS_DIR/web_viewer && python3 control_server.py 8080"
echo "  then open http://localhost:8080"
echo

# -s: server-only, no native GUI window. That GUI needs a working X/Wayland
# display (WSL's is often absent or flaky) and isn't needed here anyway --
# the browser viewer (web_viewer/) is the intended way to see the sim.
exec gz sim -s -r -v4 hirrus_runway.sdf
