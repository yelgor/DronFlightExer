from setup import connect_to_autopilot, set_guided_mode, arm_vehicle
from get_send import request_local_position_stream
from altitude_controller import reach_and_hold_altitude
from altitude_controller import AltitudePDController
from landing import land_and_wait


def main() -> None:
    connection = None
    armed = False
    landing_completed = False

    try:
        connection = connect_to_autopilot("udpin:0.0.0.0:14550")

        request_local_position_stream(
            connection,
            frequency_hz=20.0,
        )

        controller = AltitudePDController(
            kp=0.04,
            kd=0.14,
            hover_thrust=0.332,
            min_thrust=0.20,
            max_thrust=0.60,
        )

        set_guided_mode(connection)
        arm_vehicle(connection)
        armed = True

        print("Stage 1: climb to 15 m and hold.")

        reach_and_hold_altitude(
            connection=connection,
            controller=controller,
            target_altitude=15.0,
        )

        print("Stage 2: descend to 10 m and hold.")

        reach_and_hold_altitude(
            connection=connection,
            controller=controller,
            target_altitude=10.0,
        )

        print("Altitude sequence completed. Starting landing.")

        land_and_wait(
            connection,
            timeout=120.0,
        )

        landing_completed = True
        armed = False

        print("Mission completed successfully.")

    except KeyboardInterrupt:
        print("\nExecution interrupted by user.")

    except Exception as error:
        print(f"Flight error: {error}")

    finally:
        if (
            connection is not None
            and armed
            and not landing_completed
        ):
            print("Attempting automatic landing...")

            try:
                land_and_wait(
                    connection,
                    timeout=120.0,
                )

                print("Automatic landing completed.")

            except Exception as landing_error:
                print(
                    "Automatic landing failed: "
                    f"{landing_error}"
                )

        if connection is not None:
            connection.close()
            print("MAVLink connection closed.")


if __name__ == "__main__":
    main()
