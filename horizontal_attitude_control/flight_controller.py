import math
import time

from attitude_command import compensate_tilt_thrust, send_attitude_target
from config import *
from pid_controller import PIDController, clamp


class DroneFlightController:
    def __init__(self, connection, telemetry, logger):
        self.connection = connection
        self.telemetry = telemetry
        self.logger = logger
        self.started_at = time.monotonic()

        self.altitude_pid = PIDController(
            ALTITUDE_KP,
            ALTITUDE_KI,
            ALTITUDE_KD,
            MIN_THRUST - HOVER_THRUST,
            MAX_THRUST - HOVER_THRUST,
        )

        max_pitch = math.radians(MAX_PITCH_DEG)

        self.forward_pid = PIDController(
            FORWARD_KP,
            FORWARD_KI,
            FORWARD_KD,
            -max_pitch,
            max_pitch,
        )

    def calculate_vertical_thrust(self, target_altitude, state, dt):
        correction = self.altitude_pid.calculate(
            target_altitude - state["altitude"],
            state["velocity_up"],
            dt,
        )

        return clamp(
            HOVER_THRUST + correction,
            MIN_THRUST,
            MAX_THRUST,
        )

    def hold_altitude(self, target_altitude):
        self.altitude_pid.reset()

        period = 1.0 / CONTROL_FREQUENCY_HZ
        state = self.telemetry.read()
        target_yaw = state["yaw"]

        stage_started_at = time.monotonic()
        previous_time = stage_started_at
        next_log_time = stage_started_at
        stable_since = None

        while True:
            loop_started_at = time.monotonic()
            state = self.telemetry.read()
            dt = max(loop_started_at - previous_time, 0.001)
            previous_time = loop_started_at

            thrust = self.calculate_vertical_thrust(target_altitude, state, dt)
            send_attitude_target(
                self.connection,
                0.0,
                0.0,
                target_yaw,
                thrust,
            )

            altitude_error = target_altitude - state["altitude"]
            stable = (
                abs(altitude_error) <= ALTITUDE_TOLERANCE_M
                and abs(state["velocity_up"]) <= VERTICAL_SPEED_TOLERANCE_M_S
            )

            now = time.monotonic()
            stable_since = now if stable and stable_since is None else stable_since

            if not stable:
                stable_since = None

            stable_duration = 0.0 if stable_since is None else now - stable_since

            self.logger.write(
                now - self.started_at,
                "altitude",
                target_altitude,
                state["altitude"],
                0.0,
                0.0,
                0.0,
                state["velocity_up"],
                0.0,
                thrust,
            )

            if now >= next_log_time:
                print(
                    f"alt_target={target_altitude:5.2f} "
                    f"alt={state['altitude']:5.2f} "
                    f"vz={state['velocity_up']:+5.2f} "
                    f"thrust={thrust:.3f} "
                    f"hold={stable_duration:.1f}/{ALTITUDE_HOLD_DURATION_S:.1f}"
                )
                next_log_time = now + 0.25

            if stable_duration >= ALTITUDE_HOLD_DURATION_S:
                return

            if now - stage_started_at >= ALTITUDE_STAGE_TIMEOUT_S:
                raise TimeoutError("Altitude stage timeout")

            sleep_duration = period - (time.monotonic() - loop_started_at)

            if sleep_duration > 0.0:
                time.sleep(sleep_duration)

    def move_forward(self, target_altitude, distance_m):
        self.altitude_pid.reset()
        self.forward_pid.reset()

        period = 1.0 / CONTROL_FREQUENCY_HZ
        initial_state = self.telemetry.read()

        start_north = initial_state["north"]
        start_east = initial_state["east"]
        target_yaw = initial_state["yaw"]

        stage_started_at = time.monotonic()
        previous_time = stage_started_at
        next_log_time = stage_started_at
        stable_since = None

        while True:
            loop_started_at = time.monotonic()
            state = self.telemetry.read()
            dt = max(loop_started_at - previous_time, 0.001)
            previous_time = loop_started_at

            delta_north = state["north"] - start_north
            delta_east = state["east"] - start_east

            forward_position = (
                math.cos(target_yaw) * delta_north
                + math.sin(target_yaw) * delta_east
            )

            forward_speed = (
                math.cos(target_yaw) * state["velocity_north"]
                + math.sin(target_yaw) * state["velocity_east"]
            )

            forward_error = distance_m - forward_position

            pitch_command = -self.forward_pid.calculate(
                forward_error,
                forward_speed,
                dt,
            )

            vertical_thrust = self.calculate_vertical_thrust(
                target_altitude,
                state,
                dt,
            )

            thrust = compensate_tilt_thrust(
                vertical_thrust,
                0.0,
                pitch_command,
                MIN_THRUST,
                MAX_THRUST,
            )

            send_attitude_target(
                self.connection,
                0.0,
                pitch_command,
                target_yaw,
                thrust,
            )

            stable = (
                abs(forward_error) <= POSITION_TOLERANCE_M
                and abs(forward_speed) <= FORWARD_SPEED_TOLERANCE_M_S
                and abs(target_altitude - state["altitude"]) <= ALTITUDE_TOLERANCE_M
                and abs(state["velocity_up"]) <= VERTICAL_SPEED_TOLERANCE_M_S
            )

            now = time.monotonic()
            stable_since = now if stable and stable_since is None else stable_since

            if not stable:
                stable_since = None

            stable_duration = 0.0 if stable_since is None else now - stable_since

            self.logger.write(
                now - self.started_at,
                "forward",
                target_altitude,
                state["altitude"],
                distance_m,
                forward_position,
                forward_speed,
                state["velocity_up"],
                math.degrees(pitch_command),
                thrust,
            )

            if now >= next_log_time:
                print(
                    f"x_target={distance_m:5.2f} "
                    f"x={forward_position:5.2f} "
                    f"vx={forward_speed:+5.2f} "
                    f"alt={state['altitude']:5.2f} "
                    f"pitch={math.degrees(pitch_command):+5.2f} "
                    f"hold={stable_duration:.1f}/{POSITION_HOLD_DURATION_S:.1f}"
                )
                next_log_time = now + 0.25

            if stable_duration >= POSITION_HOLD_DURATION_S:
                return

            if now - stage_started_at >= FORWARD_STAGE_TIMEOUT_S:
                raise TimeoutError("Forward stage timeout")

            sleep_duration = period - (time.monotonic() - loop_started_at)

            if sleep_duration > 0.0:
                time.sleep(sleep_duration)
