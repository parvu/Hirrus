#!/bin/bash
# Launch gz-sim for the Hirrus flying wing on the runway world (the
# active dev target -- runway alignment + launch-ramp/catapult tuning).
# Full launch sequence (SITL, browser viewer) is in README.md -- this
# script only does the gz sim half.
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

# -r: start running immediately (some worlds default to paused, e.g.
# Domnesti-Slanic-5.world's own GUI config sets <start_paused>true</start_paused>
# -- without -r the sim just sits frozen, physics never steps, and
# arduplane loops "No JSON sensor message received" forever, looking like
# a connection problem when it's actually just a paused sim).
WORLD_FILE="$HIRRUS_DIR/worlds/hirrus_runway.sdf"

# --no-sitl: gz sim only, launch ArduPilot yourself (or from Mission
# Planner). Default: also start ArduPilot SITL against this gz world.
START_SITL=1
GZ_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --no-sitl) START_SITL=0 ;;
    *) GZ_ARGS+=("$arg") ;;
  esac
done

if [ "$START_SITL" -eq 0 ]; then
  exec gz sim -r -v4 "$WORLD_FILE" "${GZ_ARGS[@]}"
fi

# gz sim in the BACKGROUND; sim_vehicle.py in the FOREGROUND. sim_vehicle
# runs MAVProxy, an interactive console app -- if it's backgrounded it has
# no controlling terminal, reads EOF on stdin, and exits immediately
# ("MAVProxy exited" -> "Killing tasks"), which then kills arduplane and,
# via the trap, gz. So it MUST stay in the foreground. When it exits
# (you Ctrl-C the MAVProxy console), the EXIT trap stops gz.
gz sim -s -r -v4 "$WORLD_FILE" "${GZ_ARGS[@]}" &
GZ_PID=$!
trap 'kill "$GZ_PID" 2>/dev/null' EXIT INT TERM

# Let gz bind its transport + the ArduPilotPlugin's FDM socket (9002)
# before SITL starts hammering it.
sleep 5

# sim_vehicle.py isn't on PATH -- it lives in the ardupilot checkout.
# --custom-location matches worlds/hirrus_runway.sdf's <spherical_coordinates>
# (lat,lon,alt,heading). --no-rebuild: don't recompile every run. Add
# --no-mavproxy if you only ever connect from Mission Planner / the web
# viewer (arduplane still serves TCP 5760/5762/5763) and don't want the
# console.
#"$HOME/ardupilot/Tools/autotest/sim_vehicle.py" -v ArduPlane --model JSON -f gazebo-zephyr\
"$HOME/ardupilot/Tools/autotest/sim_vehicle.py" -v ArduPlane --model JSON --wipe --frame gazebo-zephyr --console \
  --add-param-file="$HIRRUS_DIR/hirrus.parm" \
  --custom-location=44.39361401276187,25.941499548422655,85.2,0 \
  --no-rebuild
