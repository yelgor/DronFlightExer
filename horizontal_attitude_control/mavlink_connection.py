import time
from pymavlink import mavutil


def connect_to_autopilot(connection_string):
    print(f"Connecting to {connection_string}")
    connection = mavutil.mavlink_connection(connection_string, autoreconnect=True)
    print("Waiting for heartbeat")
    connection.wait_heartbeat(timeout=30)

    connection.target_system = connection.target_system or 1
    connection.target_component = (
        connection.target_component or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
    )

    print(
        f"Connected: system={connection.target_system}, "
        f"component={connection.target_component}"
    )
    return connection


def request_message_interval(connection, message_id, frequency_hz):
    interval_us = int(1_000_000.0 / float(frequency_hz))

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        int(message_id),
        interval_us,
        0, 0, 0, 0, 0,
    )


def request_telemetry(connection, frequency_hz):
    request_message_interval(
        connection,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        frequency_hz,
    )
    request_message_interval(
        connection,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        frequency_hz,
    )
    print(f"Telemetry requested at {frequency_hz:.1f} Hz")


def set_mode(connection, mode_name, timeout=10.0):
    mode_mapping = connection.mode_mapping()

    if mode_name not in mode_mapping:
        raise RuntimeError(f"{mode_name} mode is unavailable")

    mode_id = mode_mapping[mode_name]

    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    deadline = time.monotonic() + float(timeout)

    while time.monotonic() < deadline:
        message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)

        if message is not None and int(message.custom_mode) == int(mode_id):
            print(f"{mode_name} mode confirmed")
            return

    raise TimeoutError(f"{mode_name} mode was not confirmed")


def arm_vehicle(connection, timeout=15.0):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0, 0, 0, 0, 0, 0,
    )

    deadline = time.monotonic() + float(timeout)

    while time.monotonic() < deadline:
        message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)

        if message is None:
            continue

        armed = bool(
            message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        if armed:
            print("Vehicle armed")
            return

    raise TimeoutError("Vehicle was not armed")


def land_and_wait(connection, timeout=120.0):
    set_mode(connection, "LAND", timeout=10.0)
    deadline = time.monotonic() + float(timeout)

    while time.monotonic() < deadline:
        message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)

        if message is None:
            continue

        armed = bool(
            message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        if not armed:
            print("Vehicle landed and disarmed")
            return

    raise TimeoutError("Landing timeout")
