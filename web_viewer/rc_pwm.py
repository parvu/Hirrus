"""RC-override PWM mapping and ArduPlane mode constants, shared by
control_server.py. Standalone (no ROS2/MAVROS) -- ported from
~/AeroCanyon_Guard/src/aerocanyon/aerocanyon/rc_pwm.py, whose pwm/
pwm_throttle/resolve_stick were already pure functions with no rclpy
dependency; only arm()/set_mode() are rewritten here against pymavlink
directly instead of mavros_msgs service clients.
"""
# RC override PWM: 1500 centre, +/-500 at full stick deflection, the
# standard 1000-2000 us band ArduPilot's RCn_MIN/MAX default to.
RC_CENTER = 1500
RC_SPAN = 500
# Throttle's own PWM range: caps at 1900, not 2000 (RC_CENTER + RC_SPAN),
# matching a real Mode 2 transmitter's ratcheted throttle gimbal.
THROTTLE_MID = 1450
THROTTLE_SPAN = 450

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
