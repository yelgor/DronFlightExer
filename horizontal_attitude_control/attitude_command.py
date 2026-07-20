import math
import time
from pid_controller import clamp


def euler_to_quaternion(roll, pitch, yaw):
    roll *= 0.5
    pitch *= 0.5
    yaw *= 0.5

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def send_attitude_target(connection, roll, pitch, yaw, thrust):
    quaternion = euler_to_quaternion(roll, pitch, yaw)
    time_boot_ms = int(time.monotonic() * 1000.0) & 0xFFFFFFFF

    connection.mav.set_attitude_target_send(
        time_boot_ms,
        connection.target_system,
        connection.target_component,
        7,
        quaternion,
        0.0,
        0.0,
        0.0,
        clamp(float(thrust), 0.0, 1.0),
    )


def compensate_tilt_thrust(vertical_thrust, roll, pitch, minimum, maximum):
    denominator = max(math.cos(roll) * math.cos(pitch), 0.5)
    return clamp(vertical_thrust / denominator, minimum, maximum)
