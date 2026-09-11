#!/bin/bash
# Browser viewer for the Hirrus flying wing: the gz-launch websocket bridge
# (scene stream) + the pymavlink control server (RC-override manual flight).
# Run AFTER run_sitl.sh -- both halves attach to an already-running gz sim +
# arduplane SITL, they don't start one.
set -uo pipefail

HIRRUS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$HIRRUS_DIR/models:$HIRRUS_DIR/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi

cd "$HIRRUS_DIR/web_viewer" || exit 1

# gz-launch isn't a registered `gz` subcommand on the ROS-vendored CLI here
# -- call the binary directly (positional arg, no -f).
GZ_LAUNCH_BIN=/usr/lib/x86_64-linux-gnu/gz/launch7/gz-launch

"$GZ_LAUNCH_BIN" websocket.gzlaunch &
GZL_PID=$!
# Kill the websocket bridge when this script exits, so a re-run doesn't
# stack a second one on the same port.
trap 'kill "$GZL_PID" 2>/dev/null' EXIT INT TERM

sleep 2

# control_server stays in the foreground (this is the process you Ctrl-C to
# stop the viewer). MAVLink port 5763 (SERIAL2): run_sitl.sh's MAVProxy
# owns 5760, so the viewer uses a spare serial port instead of contending
# for the same socket. 5763 is open because MAVProxy has connected to 5760.
exec python3 control_server.py "${1:-8080}" --mavlink=tcp:127.0.0.1:5763 >/dev/null 2>&1
