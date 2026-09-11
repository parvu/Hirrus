"""RC-override PWM mapping and ArduPlane mode constants, shared by
control_server.py. Standalone (no ROS2/MAVROS) -- ported from
~/AeroCanyon_Guard/src/aerocanyon/aerocanyon/rc_pwm.py, whose pwm/
pwm_throttle/resolve_stick were already pure functions with no rclpy
dependency; only arm()/set_mode() are rewritten here against pymavlink
directly instead of mavros_msgs service clients.
"""
# RC override PWM: 1500 centre, +/-400 at full stick deflection -- matches
# hirrus.parm's RC1/2/3_MIN/MAX (1100/1900), not the raw 1000-2000 band a
# receiver could physically produce. This used to be +/-500 (1000-2000),
# which overshot RCn_MIN/MAX -- ArduPilot's norm_input() clamps that to the
# same +/-1.0 either way, so full stick still worked, but the *reported*
# RC/servo values never matched a "calibrated" 1100-1900 transmitter,
# which is what was actually being asked for.
RC_CENTER = 1500
RC_SPAN = 400
# Throttle's own PWM range: same 1100-1900 calibrated band as the other
# three channels (a real Mode 2 transmitter's ratcheted throttle gimbal
# still travels its full physical range, it just isn't self-centering).
THROTTLE_MID = 1500
THROTTLE_SPAN = 400

# ArduPlane custom mode numbers (ArduPlane/mode.h). No Q-modes here --
# hirrus.parm configures a plain fixed-wing ArduPlane, not QuadPlane.
MODE_MANUAL = 0
MODE_CIRCLE = 1
MODE_FLY_BY_WIRE_A = 5
MODE_AUTO = 10
MODE_RTL = 11

MAV_CMD_DO_SET_MODE = 176
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_CMD_COMPONENT_ARM_DISARM = 400


def resolve_stick(stick, stick_time, now, timeout):
    """Per-axis dead-man's-switch: an axis whose last update is older
    than `timeout` reads as 0.0 regardless of its last commanded value."""
    return {
        name: (0.0 if now - stick_time[name] > timeout else stick[name])
        for name in stick
    }


def pwm(value, scale, invert=False):
    """A [-1, 1] command axis -> RC_CENTER +/- RC_SPAN PWM."""
    v = max(-1.0, min(1.0, value * scale))
    return int(round(RC_CENTER + (-v if invert else v) * RC_SPAN))


def pwm_throttle(value, scale):
    """A [-1, 1] throttle command -> THROTTLE_MID +/- THROTTLE_SPAN PWM."""
    v = max(-1.0, min(1.0, value * scale))
    return int(round(THROTTLE_MID + v * THROTTLE_SPAN))


def arm(master, value):
    """Request arm (value=True) or disarm (value=False) over an already-
    connected pymavlink mavutil connection."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1 if value else 0, 0, 0, 0, 0, 0, 0)


def set_mode(master, mode):
    """Request a custom-mode switch (e.g. MODE_FLY_BY_WIRE_A) over an
    already-connected pymavlink mavutil connection. Goes through
    COMMAND_LONG/DO_SET_MODE (not the deprecated SET_MODE message) --
    same approach AeroCanyon_Guard's own rc_pwm.set_mode uses via MAVROS,
    ported here to raw MAVLink."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        MAV_CMD_DO_SET_MODE, 0,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, float(mode), 0, 0, 0, 0, 0)
