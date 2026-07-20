import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from config import CSV_FILE


class LiveFlightPlot:
    def __init__(self, filename):
        self.path = Path(filename)

        self.figure, self.axes = plt.subplots(2, 1, figsize=(11, 8))
        self.altitude_axis = self.axes[0]
        self.forward_axis = self.axes[1]

        self.altitude_setpoint_line, = self.altitude_axis.plot([], [], label="Altitude setpoint")
        self.altitude_line, = self.altitude_axis.plot([], [], label="Current altitude")

        self.forward_setpoint_line, = self.forward_axis.plot([], [], label="Forward setpoint")
        self.forward_line, = self.forward_axis.plot([], [], label="Current forward position")

        self.altitude_axis.set_title("Altitude control")
        self.altitude_axis.set_xlabel("Time, s")
        self.altitude_axis.set_ylabel("Altitude, m")
        self.altitude_axis.grid(True)
        self.altitude_axis.legend()

        self.forward_axis.set_title("Forward position control")
        self.forward_axis.set_xlabel("Time, s")
        self.forward_axis.set_ylabel("Forward position, m")
        self.forward_axis.grid(True)
        self.forward_axis.legend()

        self.figure.tight_layout()

    def read_data(self):
        if not self.path.exists():
            return None

        try:
            with self.path.open("r", encoding="utf-8", newline="") as file:
                rows = list(csv.DictReader(file))
        except (OSError, csv.Error):
            return None

        if not rows:
            return None

        data = {
            "time": [],
            "target_altitude": [],
            "altitude": [],
            "target_forward": [],
            "forward": [],
        }

        for row in rows:
            try:
                data["time"].append(float(row["time_s"]))
                data["target_altitude"].append(float(row["target_altitude_m"]))
                data["altitude"].append(float(row["altitude_m"]))
                data["target_forward"].append(float(row["target_forward_m"]))
                data["forward"].append(float(row["forward_m"]))
            except (KeyError, TypeError, ValueError):
                continue

        if not data["time"]:
            return None

        return data

    def update(self, _):
        data = self.read_data()

        if data is None:
            return (
                self.altitude_setpoint_line,
                self.altitude_line,
                self.forward_setpoint_line,
                self.forward_line,
            )

        self.altitude_setpoint_line.set_data(data["time"], data["target_altitude"])
        self.altitude_line.set_data(data["time"], data["altitude"])

        self.forward_setpoint_line.set_data(data["time"], data["target_forward"])
        self.forward_line.set_data(data["time"], data["forward"])

        self.altitude_axis.relim()
        self.altitude_axis.autoscale_view()

        self.forward_axis.relim()
        self.forward_axis.autoscale_view()

        return (
            self.altitude_setpoint_line,
            self.altitude_line,
            self.forward_setpoint_line,
            self.forward_line,
        )

    def run(self):
        self.animation = FuncAnimation(
            self.figure,
            self.update,
            interval=250,
            cache_frame_data=False,
        )

        plt.show()


def main():
    plot = LiveFlightPlot(CSV_FILE)
    plot.run()


if __name__ == "__main__":
    main()
