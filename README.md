# Hirrus

Gazebo/ArduPilot SITL for the Hirrus flying wing: a fixed-wing (pusher prop
+ 2 elevons + tricycle landing gear) `gz-sim` model built from
`data/Hirrus.avl` and `data/Hirrus.dae`, flying from `worlds/hirrus_runway.sdf`
(a flat runway world -- the active dev target; `worlds/Domnesti-Slanic-5`
is a real-terrain alternative, see its own Known gaps note below), plus a
browser-based viewer with manual controls (no ROS2/MAVROS dependency).

## Prerequisites

Same as `~/AeroCanyon_Guard`'s own prerequisites (this project shares that
environment):

- ROS2 Jazzy + Gazebo Harmonic: `sudo apt install ros-jazzy-ros-gz*`
- **[ArduPilot](https://github.com/ArduPilot/ardupilot)** cloned
  `--recursive` at `$HOME/ardupilot`, built for SITL:
  `./waf configure --board sitl && ./waf plane`
- **[ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)**
  cloned at `$HOME/ardupilot_gazebo`, built against Gazebo Harmonic:
  `cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j$(nproc)`

## Fly it

Four terminals. `run_sitl.sh` only starts `gz sim`; SITL and the browser
viewer are separate processes so any of them can be restarted
independently.

```bash
# Terminal 1: Gazebo (headless -- see run_sitl.sh's own comment for why)
./run_sitl.sh
```

```bash
# Terminal 2: ArduPilot SITL. sim_vehicle.py isn't on PATH by default --
# it lives inside the ardupilot checkout. --custom-location matches
# worlds/hirrus_runway.sdf's own <spherical_coordinates> (lat,lon,alt,
# heading); leaving it out would default to CMAC, on the other side of
# the planet from where the runway actually is.
$HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane --model JSON \
  --add-param-file=$HOME/Hirrus/hirrus.parm \
  --custom-location=44.39361401276187,25.941499548422655,85.2,0 \
  --map --console
```

```bash
# Terminal 3: browser viewer's Gazebo websocket bridge (scene stream).
# `gz launch` isn't a registered subcommand on this system's (ROS-vendored)
# `gz` CLI -- call the actual binary directly instead (positional arg, no
# -f flag).
source /opt/ros/jazzy/setup.bash
cd $HOME/Hirrus/web_viewer
/usr/lib/x86_64-linux-gnu/gz/launch7/gz-launch websocket.gzlaunch
```

```bash
# Terminal 4: control server (static files + RC-override manual control).
# Plain pymavlink, no ROS2/MAVROS -- talks to SITL's MAVLink TCP port
# (5760) directly.
cd $HOME/Hirrus/web_viewer
python3 control_server.py 8080
```

Open `http://localhost:8080`. Arm, switch to `FBWA`, throttle up, and it
rolls down the runway on its own tricycle gear -- the on-screen sticks are
Mode 2 (left: yaw + climb, right: roll + pitch).

No catapult/launch-impulse mechanism anymore, on purpose: it caused
tumbling and an uncommanded-thrust risk (see git history on
`models/launch_ramp` and `model.sdf`'s own gear comment) and real landing
gear makes it unnecessary -- takeoff is just commanded throttle now, same
as flying any other wheeled ArduPlane frame. There's also no
auto-throttle-on-launch-detection in `hirrus.parm` (removed
`TKOFF_THR_MINACC` and friends) -- throttle only ever comes from an
explicit stick/RC command.

## Mission Planner / a second GCS

SITL exposes three MAVLink TCP ports: **5760** (SERIAL0, also used by
`control_server.py` -- confirmed live that it accepts multiple simultaneous
clients, so this is safe to share), 5762 and 5763 (SERIAL1/2). The latter
two don't open until *something* has connected to 5760 first (ArduPilot's
own boot sequence gates on it) -- just use 5760 unless you specifically
need a second, independent connection.

## Known gaps

- No measured stall speed for this airframe -- `data/Polara.txt`'s own
  AVL sweep only covers alpha<=8.4deg (normal cruise trim), so
  `model.sdf`'s stall-region `LiftDrag` coefficients (`alpha_stall`,
  `cla_stall`, `cda_stall`) are still placeholders carried over from the
  tricopter VTOL variant, not measured for this airframe.
- `Domnesti-Slanic-5.world` (the real-terrain alternative, not the
  default) still spawns hirrus at a fixed guessed Z (currently 90) --
  the heightmap collision's exact elevation at that XY isn't readable
  off its SDF, so this can still land the plane inside the terrain
  rather than on it. A `models/launch_ramp` platform was tried there and
  reverted (caused more problems than it solved). Not an issue on
  `hirrus_runway.sdf`, the default/active world, which has a flat runway
  plane instead.
- Landing gear (nose + 2 main wheels) is a first-pass layout (positions,
  wheel sizes) verified live to settle stably and hold the airframe clear
  of the ground -- not yet flown through an actual takeoff roll to check
  taxi/rotation behaviour.
