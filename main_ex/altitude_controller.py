import math
import time

from get_send import read_vertical_state, send_thrust

# Налаштування виконання етапу.
# Це не параметри PD і не цільова висота.
CONTROL_FREQUENCY_HZ = 20.0

ALTITUDE_TOLERANCE_M = 0.25
VERTICAL_SPEED_TOLERANCE_M_S = 0.20

HOLD_DURATION_S = 18.0
STAGE_TIMEOUT_S = 90.0
class AltitudePDController:
    def __init__(self, kp: float, kd: float, hover_thrust: float, min_thrust = 0.0,
                 max_thrust = 1.0) -> None:

        if kp < 0.0:
            raise ValueError("kp must be non-negative")

        if kd < 0.0:
            raise ValueError("kd must be non-negative")

        if not 0.0 <= min_thrust < max_thrust <= 1.0:
            raise ValueError(
                "Thrust limits must satisfy "
                "0 <= min_thrust < max_thrust <= 1"
            )

        if not min_thrust <= hover_thrust <= max_thrust:
            raise ValueError(
                "hover_thrust must be inside thrust limits"
            )

        self.kp = kp
        self.kd = kd

        self.hover_thrust = hover_thrust
        self.min_thrust = min_thrust
        self.max_thrust = max_thrust


        self.last_error = 0.0
        self.last_p_term = 0.0
        self.last_d_term = 0.0
        self.last_raw_thrust = hover_thrust
        self.last_thrust = hover_thrust

    def calculate_thrust(self, target_altitude: float, current_altitude: float, vertical_speed_up: float) -> float:
        error = target_altitude - current_altitude

        p_term = self.kp * error
        d_term = -self.kd * vertical_speed_up

        raw_thrust = (
            self.hover_thrust
            + p_term
            + d_term
        )

        thrust = max(
            self.min_thrust,
            min(self.max_thrust, raw_thrust),
        )

        self.last_error = error
        self.last_p_term = p_term
        self.last_d_term = d_term
        self.last_raw_thrust = raw_thrust
        self.last_thrust = thrust

        return thrust

def reach_and_hold_altitude(connection, controller, target_altitude):
    """
    Bring the drone to target_altitude and hold it there.

    The function returns only after the drone has continuously remained
    inside the altitude and vertical-speed tolerances for HOLD_DURATION_S.
    """

    if not math.isfinite(target_altitude):
        raise ValueError("target_altitude must be finite")

    if target_altitude < 0.0:
        raise ValueError("target_altitude must be non-negative")

    control_period = 1.0 / CONTROL_FREQUENCY_HZ

    stage_started_at = time.monotonic()
    stable_since = None
    next_log_time = stage_started_at

    print(
        f"\nStarting altitude stage: "
        f"target={target_altitude:.2f} m"
    )

    while True:
        loop_started_at = time.monotonic()

        # 1. Отримуємо фактичний стан дрона.
        current_altitude, vertical_speed_up = (
            read_vertical_state(
                connection=connection,
                time_out=8.0,
            )
        )

        # 2. Контролер обчислює новий thrust.
        thrust = controller.calculate_thrust(
            target_altitude=target_altitude,
            current_altitude=current_altitude,
            vertical_speed_up=vertical_speed_up,
        )

        # 3. Надсилаємо команду ArduPilot.
        send_thrust(
            connection=connection,
            thrust=thrust,
        )

        # 4. Перевіряємо стабільність.
        # Помилку не рахуємо повторно:
        # беремо значення, яке вже отримав контролер.
        altitude_is_stable = (
            abs(controller.last_error)
            <= ALTITUDE_TOLERANCE_M
        )

        vertical_speed_is_stable = (
            abs(vertical_speed_up)
            <= VERTICAL_SPEED_TOLERANCE_M_S
        )

        drone_is_stable = (
            altitude_is_stable
            and vertical_speed_is_stable
        )

        current_time = time.monotonic()

        # 5. Керуємо таймером утримання.
        if drone_is_stable:
            if stable_since is None:
                stable_since = current_time

                print(
                    f"Drone entered stable region near "
                    f"{target_altitude:.2f} m"
                )

            stable_duration = current_time - stable_since

            if stable_duration >= HOLD_DURATION_S:
                print(
                    f"Altitude {target_altitude:.2f} m "
                    f"was held for {HOLD_DURATION_S:.1f} s"
                )
                return

        else:
            # Якщо дрон вийшов із допустимої області,
            # відлік утримання починається заново.
            if stable_since is not None:
                print(
                    "Drone left the stable region. "
                    "Hold timer has been reset."
                )

            stable_since = None
            stable_duration = 0.0

        # 6. Діагностика, приблизно 4 рази на секунду.
        if current_time >= next_log_time:
            if stable_since is None:
                stable_text = "not stable"
            else:
                stable_text = (
                    f"stable {stable_duration:.2f}/"
                    f"{HOLD_DURATION_S:.2f} s"
                )

            print(
                f"target={target_altitude:6.2f} m | "
                f"alt={current_altitude:6.2f} m | "
                f"speed={vertical_speed_up:+6.2f} m/s | "
                f"error={controller.last_error:+6.2f} m | "
                f"P={controller.last_p_term:+7.3f} | "
                f"D={controller.last_d_term:+7.3f} | "
                f"thrust={thrust:5.3f} | "
                f"{stable_text}"
            )

            next_log_time = current_time + 0.25

        # 7. Захист від нескінченного циклу.
        stage_duration = current_time - stage_started_at

        if stage_duration >= STAGE_TIMEOUT_S:
            raise TimeoutError(
                f"Could not reach and hold "
                f"{target_altitude:.2f} m within "
                f"{STAGE_TIMEOUT_S:.1f} seconds"
            )

        # 8. Підтримуємо приблизно 20 Гц.
        loop_duration = time.monotonic() - loop_started_at
        sleep_duration = control_period - loop_duration

        if sleep_duration > 0.0:
            time.sleep(sleep_duration)