import time
from pymavlink import mavutil


def connect_to_autopilot(connection_string: str, time_out: float = 15.0):
    print(f"Attempting to connect to {connection_string}")
    connection = mavutil.mavlink_connection(connection_string)

    print("Waiting for HEARTBEAT message...")
    hearbeat = connection.wait_heartbeat(timeout=time_out)

    if hearbeat is None:
        raise TimeoutError(f"Heartbeat timed out after {time_out}")

    target_system = connection.target_system
    target_component = connection.target_component

    if target_system == 0:
        raise RuntimeError(
            "Failed to define autopilot system ID: "
            f"system={target_system}"
        )

    if target_component == 0:
        target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        connection.target_component = target_component

        print(
            "Autopilot component ID was not specified; "
            f"using component={target_component}"
        )

    print(
        "ArduPilot is found: "
        f"system={target_system}, "
        f"component={target_component}"
    )

    return connection


def wait_command_ack(connection, expected_command: int, timeout_s: float = 5.0):
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        ack = connection.recv_match(
            type="COMMAND_ACK",
            blocking=True,
            timeout=1.0,
        )

        if ack is None:
            continue

        if ack.command != expected_command:
            continue

        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(
                "ArduPilot rejected the command: "
                f"command={expected_command}, "
                f"result={ack.result}"
            )

        return ack

    raise TimeoutError(
        "COMMAND_ACK was not received: "
        f"command={expected_command}, "
        f"timeout={timeout_s} seconds"
    )


def set_guided_mode(connection) -> None:
    mode_mapping = connection.mode_mapping()

    if not mode_mapping:
        raise RuntimeError(
            "Failed to get the ArduPilot mode table"
        )

    if "GUIDED" not in mode_mapping:
        raise RuntimeError(
            "GUIDED mode was not found in the mode table"
        )

    guided_mode_id = mode_mapping["GUIDED"]

    print(f"GUIDED mode ID: {guided_mode_id}")
    print("Sending GUIDED mode command...")

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,  # confirmation
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        guided_mode_id,
        0,
        0,
        0,
        0,
        0,
    )

    wait_command_ack(
        connection,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    )

    print("The GUIDED mode command was accepted")

    heartbeat = connection.recv_match(
        type="HEARTBEAT",
        blocking=True,
        timeout=5,
    )

    if heartbeat is None:
        raise TimeoutError(
            "HEARTBEAT was not received after changing the mode"
        )

    actual_mode = mavutil.mode_string_v10(heartbeat)

    if actual_mode != "GUIDED":
        raise RuntimeError(
            "The vehicle did not enter GUIDED mode: "
            f"actual_mode={actual_mode}"
        )

    print("GUIDED mode has been confirmed")


def arm_vehicle(connection) -> None:
    print("Sending ARM command...")

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    wait_command_ack(
        connection,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    )

    print("The ARM command was accepted")