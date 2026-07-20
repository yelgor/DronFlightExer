from __future__ import annotations

import time

from pymavlink import mavutil


def is_armed(connection) -> bool:
    heartbeat = connection.recv_match(
        type="HEARTBEAT",
        blocking=True,
        timeout=2.0,
    )

    if heartbeat is None:
        raise TimeoutError("No HEARTBEAT received.")

    return bool(
        heartbeat.base_mode
        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )


def set_land_mode(connection, timeout: float = 10.0) -> None:
    mode_mapping = connection.mode_mapping()

    if not mode_mapping:
        raise RuntimeError("Flight mode mapping is unavailable.")

    land_mode_id = mode_mapping.get("LAND")

    if land_mode_id is None:
        raise RuntimeError("LAND mode is unavailable.")

    print("Requesting LAND mode...")

    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        land_mode_id,
    )

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        heartbeat = connection.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1.0,
        )

        if heartbeat is None:
            continue

        current_mode = mavutil.mode_string_v10(heartbeat)

        print(f"Current mode: {current_mode}")

        if heartbeat.custom_mode == land_mode_id:
            print("LAND mode confirmed.")
            return

    raise TimeoutError("LAND mode was not confirmed.")


def wait_until_disarmed(
    connection,
    timeout: float = 90.0,
) -> None:
    print("Waiting for landing and automatic disarm...")

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        heartbeat = connection.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1.0,
        )

        if heartbeat is None:
            continue

        armed = bool(
            heartbeat.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        if not armed:
            print("Vehicle landed and disarmed.")
            return

    raise TimeoutError(
        "Vehicle did not disarm within landing timeout."
    )


def land_and_wait(
    connection,
    timeout: float = 90.0,
) -> None:
    set_land_mode(connection)
    wait_until_disarmed(connection, timeout=timeout)
