#57!/usr/bin/env python3
"""control_server.py

Ported from AeroCanyon_Guard/web_viewer/control_server.py, dropped down to
plain pymavlink instead of rclpy/MAVROS: Hirrus's SITL stack has no ROS2
layer at all (see ~/Hirrus/run_sitl.sh), so this talks straight to one of
ArduPilot's own MAVLink TCP ports instead of going through mavros_node.
Default is 5763 (SERIAL2): run_sitl.sh's sim_vehicle.py runs MAVProxy on
5760 (SERIAL0), so the viewer takes a spare serial port to avoid fighting
MAVProxy over the same socket. 5762/5763 only exist once something (i.e.
MAVProxy) has connected to 5760, which run_sitl.sh guarantees.
Also dropped everything specific to AeroCanyon's tricopter VTOL (QHOVER/
QLAND/transition/stick_mixing, the wind-field topics) -- Hirrus is a plain
ArduPlane flying wing with none of that.

Manual flight still goes out as RC_CHANNELS_OVERRIDE, same reasoning as the
ported-from project: ArduPlane's GUIDED mode has no velocity-setpoint path,
so RC override (what a real transmitter sends) is the only manual-control
surface that actually moves the airframe.

MAV_GCS_SYSID gate: ArduPilot only accepts RC override (and other GCS
commands) from the system id in its MAV_GCS_SYSID parameter, default 255 --
this connects with source_system=255 for the same reason AeroCanyon_Guard's
README insists on MAVROS's system_id:=255.

Run from web_viewer/, with gz sim + arduplane SITL already up (run_sitl.sh):
    python3 control_server.py [port]
"""
import http.server
import sys
import threading
import time

from pymavlink import mavutil

from rc_pwm import (MODE_AUTO, MODE_CIRCLE, MODE_FLY_BY_WIRE_A, MODE_MANUAL,
                     MODE_RTL, arm, pwm, pwm_throttle, resolve_stick, set_mode)

_port_args = [a for a in sys.argv[1:] if not a.startswith('--')]
PORT = int(_port_args[0]) if _port_args else 8080
FDM_URL = next((a.split('=', 1)[1] for a in sys.argv[1:]
                 if a.startswith('--mavlink=')), 'tcp:127.0.0.1:5763')
CONTROL_HZ = 20
# Mode 2 RC sticks are proportional and self-centering -- the browser
# streams the live stick position continuously (see index.html) rather
# than a click-and-decay nudge. This is a dead-man's-switch: if the browser
# stops sending (closed tab, dropped connection, JS error), stale sticks
# zero out rather than freezing the last commanded value, matching real RC
# failsafe behaviour.
STICK_TIMEOUT_S = 0.3
# How long rc_bridge-style hardware input has to go quiet before the
# browser falls back to its own virtual sticks (there's no rc_bridge.py
# ported here yet -- kept for parity with the browser-side polling contract
# in case one is added later; a browser with no hardware source just never
# sets hw=1, so /api/rc_status always reports present=False, which is the
# correct default for virtual-sticks-only use).
RC_PRESENT_TIMEOUT_S = 1.0

COMMAND_COMMANDS = {'arm', 'disarm', 'auto', 'rtl', 'circle', 'fbwa', 'manual'}


def stick_to_rc(stick, scale):
    """Mode 2 stick state -> (roll, pitch, throttle, yaw) PWM for RC
    channels 1-4.

    Pitch is the one inverted axis: index.html reports "stick pushed
    forward" as +1 (up on screen) and that has to mean "fly forward", which
    on ArduPilot means nose DOWN, which is BELOW-centre PWM on channel 2 --
    ArduPilot's un-reversed pitch channel treats high PWM as nose up. Roll,
    throttle and yaw all share ArduPilot's sign already.
    """
    return (pwm(stick['roll'], scale), pwm(stick['pitch'], scale, invert=True),
            pwm_throttle(stick['throttle'], scale), pwm(stick['yaw'], scale))


class MavlinkLink:
    """Owns the pymavlink connection and the RC-override send loop. No
    ROS2 -- a plain background thread, same shape as AeroCanyon_Guard's
    WebControlNode._tick timer but driven by time.sleep instead of an
    rclpy timer."""

    def __init__(self, url):
        print(f"connecting to {url} ...")
        self.master = mavutil.mavlink_connection(url, source_system=255)
        self.master.wait_heartbeat()
        print(f"heartbeat from system {self.master.target_system} "
              f"component {self.master.target_component}")

        self._lock = threading.Lock()
        # Mode 2: left stick = yaw (x) + throttle/climb (y), right stick =
        # roll/strafe (x) + pitch/forward (y). Each in [-1, 1].
        self._stick = {'yaw': 0.0, 'throttle': 0.0, 'roll': 0.0, 'pitch': 0.0}
        self._stick_time = {'yaw': 0.0, 'throttle': 0.0, 'roll': 0.0, 'pitch': 0.0}
        self._hw_last_seen = 0.0

        threading.Thread(target=self._loop, daemon=True).start()

    def set_stick(self, hw=False, **axes):
        now = time.monotonic()
        with self._lock:
            for name, value in axes.items():
                self._stick[name] = max(-1.0, min(1.0, value))
                self._stick_time[name] = now
            if hw:
                self._hw_last_seen = now

    def rc_present(self):
        with self._lock:
            return time.monotonic() - self._hw_last_seen <= RC_PRESENT_TIMEOUT_S

    def apply_command(self, cmd):
        if cmd == 'arm':
            arm(self.master, True)
        elif cmd == 'disarm':
            arm(self.master, False)
        elif cmd == 'auto':
            set_mode(self.master, MODE_AUTO)
        elif cmd == 'rtl':
            set_mode(self.master, MODE_RTL)
        elif cmd == 'circle':
            set_mode(self.master, MODE_CIRCLE)
        elif cmd == 'fbwa':
            set_mode(self.master, MODE_FLY_BY_WIRE_A)
        elif cmd == 'manual':
            # Emergency "true" passthrough -- no stabilization, raw sticks
            # straight to the elevons/throttle, same as flipping a real
            # transmitter's mode switch.
            set_mode(self.master, MODE_MANUAL)

    def _loop(self):
        period = 1.0 / CONTROL_HZ
        while True:
            now = time.monotonic()
            with self._lock:
                stick = resolve_stick(self._stick, self._stick_time, now, STICK_TIMEOUT_S)
            roll, pitch, throttle, yaw = stick_to_rc(stick, 1.0)
            # channels 5-8 as "no change" (65535) -- this project has no
            # RC-driven mode switch channel to hold centred (unlike
            # AeroCanyon's FLTMODE_CH=0 tricopter setup).
            self.master.mav.rc_channels_override_send(
                self.master.target_system, self.master.target_component,
                roll, pitch, throttle, yaw,
                65535, 65535, 65535, 65535)
            time.sleep(period)
            # ArduPilot drops RC overrides RC_OVERRIDE_TIME (3s) after the
            # last one arrives, so this steady stream is what keeps manual
            # control alive; stopping the server hands the vehicle back to
            # its own RC.


class Handler(http.server.SimpleHTTPRequestHandler):
    link = None  # set in main()

    def do_GET(self):
        if self.path.startswith('/api/stick'):
            self._handle_stick()
        elif self.path.startswith('/api/manual'):
            self._handle_command()
        elif self.path.startswith('/api/rc_status'):
            self._handle_rc_status()
        else:
            super().do_GET()

    def _handle_stick(self):
        from urllib.parse import parse_qs, urlparse
        q = parse_qs(urlparse(self.path).query)
        present = {k: v[0] for k, v in q.items() if k in
                   ('yaw', 'throttle', 'roll', 'pitch')}
        try:
            axes = {k: float(v) for k, v in present.items()}
        except ValueError:
            self.send_response(400)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b"yaw/throttle/roll/pitch must be numeric")
            return
        if not axes:
            self.send_response(400)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b"need at least one of yaw, throttle, roll, pitch")
            return
        hw = q.get('hw', ['0'])[0] == '1'
        self.link.set_stick(hw=hw, **axes)
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def _handle_rc_status(self):
        import json
        body = json.dumps({'present': self.link.rc_present()}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_command(self):
        from urllib.parse import parse_qs, urlparse
        cmd = parse_qs(urlparse(self.path).query).get('cmd', [''])[0]
        if cmd not in COMMAND_COMMANDS:
            self.send_response(400)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(f"unknown command {cmd!r}".encode())
            return
        self.link.apply_command(cmd)
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def log_message(self, fmt, *args):
        if '/api/' in self.path:
            super().log_message(fmt, *args)


def main():
    import os
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    link = MavlinkLink(FDM_URL)
    Handler.link = link

    server = http.server.ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    print(f"control_server ready on 0.0.0.0:{PORT} (static files + /api/manual)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == '__main__':
    main()
