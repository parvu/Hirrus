# Hirrus

Gazebo/ArduPilot SITL for the Hirrus flying wing: a fixed-wing (pusher prop
+ 2 elevons, no landing gear) `gz-sim` model built from `data/Hirrus.avl`
and `data/Hirrus.dae`, flying over the Domnesti-Slanic-5 terrain tile, plus
a browser-based viewer with manual controls (no ROS2/MAVROS dependency).

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
# it lives inside the ardupilot checkout. --custom-location matches this
# terrain's own <spherical_coordinates> in
# worlds/Domnesti-Slanic-5/Domnesti-Slanic-5.world (lat,lon,alt,heading);
# leaving it out would default to CMAC, on the other side of the planet
# from where the terrain actually is.
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

Open `http://localhost:8080`. Arm, switch to `FBWA`, throttle up, then
launch (see below) -- the on-screen sticks are Mode 2 (left: yaw + climb,
right: roll + pitch).

## Launch (catapult)

No landing gear -- the model needs a push above stall speed to get
airborne (see `models/hirrus/model.sdf`'s own "Catapult launch" comment
for why this is a single manually-fired impulse rather than an automatic
timer). Once armed, in FBWA or MANUAL, throttle up, then fire it:

```bash
gz topic -t /world/Domnesti-Slanic-5/wrench -m gz.msgs.EntityWrench \
  -p 'entity: {name: "hirrus::base_link", type: LINK}, wrench: {force: {x: 130000, y: 0, z: 0}}'
```

That's a single one-physics-step (1ms) impulse sized for a ~20 m/s launch
speed (`force = mass * target_v / dt`) -- safe to re-run if the first
attempt didn't take.

## Mission Planner / a second GCS

SITL exposes three MAVLink TCP ports: **5760** (SERIAL0, also used by
`control_server.py` -- confirmed live that it accepts multiple simultaneous
clients, so this is safe to share), 5762 and 5763 (SERIAL1/2). The latter
two don't open until *something* has connected to 5760 first (ArduPilot's
own boot sequence gates on it) -- just use 5760 unless you specifically
need a second, independent connection.

## Known gaps

- Catapult launch speed/force (20 m/s target) is a reasoned estimate, not
  a measured stall speed for this airframe -- see `model.sdf`'s comment.
- `data/Polara.txt`'s own AVL sweep only covers alpha<=8.4deg (normal
  cruise trim), so `model.sdf`'s stall-region `LiftDrag` coefficients
  (`alpha_stall`, `cla_stall`, `cda_stall`) are still placeholders carried
  over from the tricopter VTOL variant, not measured for this airframe.
- Both worlds spawn hirrus on `models/launch_ramp`, a static, zero-friction
  platform, rather than directly on the terrain/runway -- avoids depending
  on the heightmap's exact elevation at the spawn XY (not readable off its
  SDF) and, on the terrain world specifically, a real Bullet-collision-
  detector bug where hirrus's own very thin `base_link_collision` box fell
  straight through anything thinner than ~2m (see `launch_ramp/model.sdf`'s
  own comment for the isolated repro). Confirmed live, settles stable on
  both worlds.
