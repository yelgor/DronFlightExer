import math
import time

from pymavlink import mavutil


def read_vertical_state(connection, time_out: float = 1.0) -> tuple[float, float]:
    message = connection.recv_match(type="LOCAL_POSITION_NED",
                                    blocking=True,
                                    timeout=time_out)

    if message is None:
        raise TimeoutError("LOCAL_POSITION_NED was not received")

    altitude_m = -float(message.z)
    vertical_speed_m_s = -float(message.vz)

    return altitude_m, vertical_speed_m_s


def request_local_position_stream(
    connection,
    frequency_hz: float = 20.0,
) -> None:
    if frequency_hz <= 0.0:
        raise ValueError("frequency_hz must be positive")

    interval_us = int(1_000_000 / frequency_hz)

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )

    print(
        f"Requested LOCAL_POSITION_NED at "
        f"{frequency_hz:.1f} Hz"
    )


def send_thrust(connection, thrust: float) -> None:
    if not math.isfinite(thrust):
        raise ValueError(
            "Thrust must be a finite number"
        )

    # Physical MAVLink range for collective thrust.
    thrust = max(0.0, min(thrust, 1.0))

    type_mask = (
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
    )

    time_boot_ms = int(
        time.monotonic() * 1000
    ) & 0xFFFFFFFF

    connection.mav.set_attitude_target_send(
        time_boot_ms,
        connection.target_system,
        connection.target_component,
        type_mask,
        [1.0, 0.0, 0.0, 0.0],  # Level attitude quaternion
        0.0,                     # Ignored roll rate
        0.0,                     # Ignored pitch rate
        0.0,                     # Ignored yaw rate
        thrust,
    )