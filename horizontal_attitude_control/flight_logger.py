import csv


class FlightLogger:
    def __init__(self, filename):
        self.file = open(filename, "w", encoding="utf-8", newline="")
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "time_s",
            "stage",
            "target_altitude_m",
            "altitude_m",
            "target_forward_m",
            "forward_m",
            "forward_speed_m_s",
            "vertical_speed_m_s",
            "pitch_command_deg",
            "thrust",
        ])

        self.file.flush()

    def write(
        self,
        time_s,
        stage,
        target_altitude,
        altitude,
        target_forward,
        forward,
        forward_speed,
        vertical_speed,
        pitch_command_deg,
        thrust,
    ):
        self.writer.writerow([
            f"{time_s:.6f}",
            stage,
            f"{target_altitude:.6f}",
            f"{altitude:.6f}",
            f"{target_forward:.6f}",
            f"{forward:.6f}",
            f"{forward_speed:.6f}",
            f"{vertical_speed:.6f}",
            f"{pitch_command_deg:.6f}",
            f"{thrust:.6f}",
        ])

        self.file.flush()

    def close(self):
        if not self.file.closed:
            self.file.close()
