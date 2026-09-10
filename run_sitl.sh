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

exec gz sim -r -v4 "$WORLD_FILE"
