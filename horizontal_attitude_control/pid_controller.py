import math


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max, integral_limit=100.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_min = float(output_min)
        self.output_max = float(output_max)
        self.integral_limit = abs(float(integral_limit))
        self.integral = 0.0
        self.last_error = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_output = 0.0

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_output = 0.0

    def calculate(self, error, derivative_measurement, dt):
        error = float(error)
        derivative_measurement = float(derivative_measurement)
        dt = float(dt)

        if not math.isfinite(error):
            raise ValueError("PID error must be finite")
        if not math.isfinite(derivative_measurement):
            raise ValueError("PID derivative measurement must be finite")
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("PID dt must be positive and finite")

        self.integral += error * dt
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        self.last_error = error
        self.last_p = self.kp * error
        self.last_i = self.ki * self.integral
        self.last_d = -self.kd * derivative_measurement

        raw_output = self.last_p + self.last_i + self.last_d
        self.last_output = clamp(raw_output, self.output_min, self.output_max)
        return self.last_output
