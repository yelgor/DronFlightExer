from __future__ import annotations

import argparse
import csv
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pymavlink import mavutil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live MAVLink altitude-over-time monitor."
    )
    parser.add_argument(
        "--connection",
        default="udpin:0.0.0.0:14551",
        help="MAVLink connection string.",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=120.0,
        help="Visible graph window in seconds.",
    )
    parser.add_argument(
        "--csv",
        default="",
        help="CSV output path. Generated automatically when omitted.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    csv_path = Path(
        args.csv
        or f"altitude_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    print(f"Connecting: {args.connection}")
    connection = mavutil.mavlink_connection(
        args.connection,
        autoreconnect=True,
    )

    heartbeat = connection.wait_heartbeat(timeout=15)

    if heartbeat is None:
        raise TimeoutError("No MAVLink heartbeat received within 15 seconds.")

    print(
        "Heartbeat received: "
        f"system={connection.target_system}, "
        f"component={connection.target_component}"
    )
    print(f"Writing CSV: {csv_path.resolve()}")

    csv_file = csv_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(
        [
            "wall_time_iso",
            "elapsed_s",
            "altitude_m",
            "vertical_speed_up_m_s",
            "x_m",
            "y_m",
            "z_ned_m",
        ]
    )
    csv_file.flush()

    samples: deque[tuple[float, float]] = deque()
    start_monotonic = time.monotonic()

    fig, ax = plt.subplots()
    line, = ax.plot([], [])
    current_point, = ax.plot([], [], marker="o")

    ax.set_title("Drone altitude over time")
    ax.set_xlabel("Time, s")
    ax.set_ylabel("Altitude, m")
    ax.grid(True)

    status_text = ax.text(
        0.02,
        0.96,
        "Waiting for LOCAL_POSITION_NED...",
        transform=ax.transAxes,
        verticalalignment="top",
    )

    latest_altitude: float | None = None
    latest_speed: float | None = None
    last_message_time = time.monotonic()

    def update(_: int):
        nonlocal latest_altitude
        nonlocal latest_speed
        nonlocal last_message_time

        received = False

        while True:
            msg = connection.recv_match(
                type="LOCAL_POSITION_NED",
                blocking=False,
            )

            if msg is None:
                break

            received = True
            elapsed = time.monotonic() - start_monotonic

            altitude = -float(msg.z)
            vertical_speed_up = -float(msg.vz)

            latest_altitude = altitude
            latest_speed = vertical_speed_up
            last_message_time = time.monotonic()

            samples.append((elapsed, altitude))

            writer.writerow(
                [
                    datetime.now().isoformat(timespec="milliseconds"),
                    f"{elapsed:.3f}",
                    f"{altitude:.4f}",
                    f"{vertical_speed_up:.4f}",
                    f"{float(msg.x):.4f}",
                    f"{float(msg.y):.4f}",
                    f"{float(msg.z):.4f}",
                ]
            )

        if received:
            csv_file.flush()

        now_elapsed = time.monotonic() - start_monotonic
        minimum_time = max(0.0, now_elapsed - args.window)

        while samples and samples[0][0] < minimum_time:
            samples.popleft()

        if samples:
            times = [sample[0] for sample in samples]
            altitudes = [sample[1] for sample in samples]

            line.set_data(times, altitudes)
            current_point.set_data([times[-1]], [altitudes[-1]])

            x_min = max(0.0, times[-1] - args.window)
            x_max = max(args.window, times[-1] + 1.0)
            ax.set_xlim(x_min, x_max)

            altitude_min = min(altitudes)
            altitude_max = max(altitudes)
            margin = max(0.5, (altitude_max - altitude_min) * 0.15)

            ax.set_ylim(
                min(0.0, altitude_min - margin),
                altitude_max + margin,
            )

        telemetry_age = time.monotonic() - last_message_time

        if latest_altitude is None:
            status_text.set_text("Waiting for LOCAL_POSITION_NED...")
        elif telemetry_age > 2.0:
            status_text.set_text(
                f"Telemetry timeout: {telemetry_age:.1f} s\n"
                f"Last altitude: {latest_altitude:.2f} m"
            )
        else:
            status_text.set_text(
                f"Altitude: {latest_altitude:.2f} m\n"
                f"Vertical speed: {latest_speed:+.2f} m/s"
            )

        return line, current_point, status_text

    animation = FuncAnimation(
        fig,
        update,
        interval=100,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.tight_layout()
        plt.show()
    finally:
        csv_file.flush()
        csv_file.close()
        connection.close()
        print(f"Saved: {csv_path.resolve()}")


if __name__ == "__main__":
    main()
