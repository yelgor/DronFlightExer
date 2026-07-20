from config import *
from flight_controller import DroneFlightController
from flight_logger import FlightLogger
from mavlink_connection import (
    arm_vehicle,
    connect_to_autopilot,
    land_and_wait,
    request_telemetry,
    set_mode,
)
from telemetry import TelemetryReader


def main():
    connection = None
    logger = None
    armed = False

    try:
        connection = connect_to_autopilot(CONNECTION_STRING)
        request_telemetry(connection, TELEMETRY_FREQUENCY_HZ)

        telemetry = TelemetryReader(connection)
        logger = FlightLogger(CSV_FILE)
        controller = DroneFlightController(connection, telemetry, logger)

        set_mode(connection, "GUIDED")
        arm_vehicle(connection)
        armed = True

        print(f"Climbing to {TARGET_ALTITUDE_M:.1f} m")
        controller.hold_altitude(TARGET_ALTITUDE_M)

        print(f"Moving forward {FORWARD_DISTANCE_M:.1f} m")
        controller.move_forward(TARGET_ALTITUDE_M, FORWARD_DISTANCE_M)

        print("Mission completed")
        land_and_wait(connection, LAND_TIMEOUT_S)
        armed = False

    except KeyboardInterrupt:
        print("Interrupted")

    except Exception as error:
        print(f"Flight error: {error}")

    finally:
        if connection is not None and armed:
            try:
                land_and_wait(connection, LAND_TIMEOUT_S)
            except Exception as error:
                print(f"Landing error: {error}")

        if logger is not None:
            logger.close()

        if connection is not None:
            connection.close()


if __name__ == "__main__":
    main()
