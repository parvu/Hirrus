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

Two terminals: `./run_sitl.sh` (gz sim + ArduPilot SITL together) and,
if you want the browser viewer, `./run_webview.sh`.

```bash
# Terminal 1: gz sim + ArduPilot SITL. Starts gz on worlds/hirrus_runway.sdf
# then sim_vehicle.py against it (--custom-location matches that world's
# <spherical_coordinates>). Ctrl-C stops both. Pass --no-sitl for gz only
# (e.g. when a SITL on the Windows host connects instead -- see the
# fdm_addr=0.0.0.0 comment in models/hirrus/model.sdf).
./run_sitl.sh
```

```bash
# Terminal 2 (optional): browser viewer -- gz-launch websocket bridge
# (scene stream) + the pymavlink control server. Attaches to the already-
# running gz + SITL from Terminal 1.
./run_webview.sh
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

`run_sitl.sh` runs `sim_vehicle.py`, whose MAVProxy owns **5760** (SERIAL0).
SITL also exposes **5762** and **5763** (SERIAL1/2), which open once
MAVProxy has connected to 5760. The web viewer's `control_server.py`
defaults to **5763** so it isn't contending with MAVProxy over 5760.
Connect Mission Planner (or any other GCS) to **5762**, or to MAVProxy's
own `udp:127.0.0.1:14550` / `14551` outputs.

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
