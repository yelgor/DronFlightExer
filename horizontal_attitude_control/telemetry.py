import math
import time


class TelemetryReader:
    def __init__(self, connection):
        self.connection = connection
        self.position_message = None
        self.attitude_message = None

    def read(self, timeout=8.0):
        deadline = time.monotonic() + float(timeout)

        while time.monotonic() < deadline:
            message = self.connection.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE"],
                blocking=True,
                timeout=0.2,
            )

            if message is None:
                continue

            message_type = message.get_type()

            if message_type == "LOCAL_POSITION_NED":
                self.position_message = message
            elif message_type == "ATTITUDE":
                self.attitude_message = message

            if self.position_message is None or self.attitude_message is None:
                continue

            position = self.position_message
            attitude = self.attitude_message

            values = [
                position.x, position.y, position.z,
                position.vx, position.vy, position.vz,
                attitude.roll, attitude.pitch, attitude.yaw,
            ]

            if not all(math.isfinite(float(value)) for value in values):
                continue

            return {
                "timestamp": time.monotonic(),
                "north": float(position.x),
                "east": float(position.y),
                "altitude": -float(position.z),
                "velocity_north": float(position.vx),
                "velocity_east": float(position.vy),
                "velocity_up": -float(position.vz),
                "roll": float(attitude.roll),
                "pitch": float(attitude.pitch),
                "yaw": float(attitude.yaw),
            }

        raise TimeoutError("Required telemetry was not received")
