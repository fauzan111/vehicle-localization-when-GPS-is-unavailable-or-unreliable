"""Generate a deterministic planar trajectory and noisy localization sensor streams."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def generate(
    config: dict[str, Any],
) -> tuple[list[dict[str, float | int]], list[dict[str, float | int]]]:
    """Return ground-truth and sensor rows for one configured scenario."""
    scenario = config["scenario"]
    sensors = config["sensors"]
    frequency = float(scenario["rate_hz"])
    duration = float(scenario["duration_s"])
    if frequency <= 0 or duration <= 0:
        raise ValueError("scenario rate_hz and duration_s must be positive")

    dt = 1.0 / frequency
    count = int(round(duration * frequency)) + 1
    time = np.arange(count, dtype=float) * dt
    motion = scenario["motion"]
    rng = np.random.default_rng(int(scenario["seed"]))

    speed = float(motion["initial_speed_mps"]) + float(motion["acceleration_mps2"]) * time
    yaw_rate = float(motion["yaw_rate_amplitude_rps"]) * np.sin(
        2.0 * np.pi * time / float(motion["yaw_rate_period_s"])
    )
    heading = np.zeros(count, dtype=float)
    x = np.zeros(count, dtype=float)
    y = np.zeros(count, dtype=float)
    for index in range(1, count):
        heading[index] = heading[index - 1] + yaw_rate[index - 1] * dt
        x[index] = x[index - 1] + speed[index - 1] * np.cos(heading[index - 1]) * dt
        y[index] = y[index - 1] + speed[index - 1] * np.sin(heading[index - 1]) * dt

    truth: list[dict[str, float | int]] = []
    sensor_rows: list[dict[str, float | int]] = []
    wheel = sensors["wheel_speed"]
    gyro = sensors["imu_gyro_z"]
    camera = sensors["camera_delta_pose"]
    gnss = sensors["gnss_position"]
    outages = [(float(start), float(end)) for start, end in gnss["outages_s"]]

    for i, timestamp in enumerate(time):
        truth.append(
            {
                "time_s": timestamp,
                "x_m": x[i],
                "y_m": y[i],
                "heading_rad": heading[i],
                "speed_mps": speed[i],
                "yaw_rate_rps": yaw_rate[i],
            }
        )
        if i == 0:
            dx = dy = d_heading = 0.0
        else:
            dx_world = x[i] - x[i - 1]
            dy_world = y[i] - y[i - 1]
            cosine, sine = np.cos(heading[i - 1]), np.sin(heading[i - 1])
            dx = cosine * dx_world + sine * dy_world
            dy = -sine * dx_world + cosine * dy_world
            d_heading = heading[i] - heading[i - 1]

        gnss_available = int(not any(start <= timestamp < end for start, end in outages))
        sensor_rows.append(
            {
                "time_s": timestamp,
                "wheel_speed_mps": speed[i]
                + float(wheel["bias_mps"])
                + rng.normal(0.0, float(wheel["noise_std_mps"])),
                "imu_gyro_z_rps": yaw_rate[i]
                + float(gyro["bias_rps"])
                + rng.normal(0.0, float(gyro["noise_std_rps"])),
                "camera_dx_body_m": dx
                + (rng.normal(0.0, float(camera["translation_noise_std_m"])) if i else 0.0),
                "camera_dy_body_m": dy
                + (rng.normal(0.0, float(camera["translation_noise_std_m"])) if i else 0.0),
                "camera_dheading_rad": d_heading
                + (rng.normal(0.0, float(camera["heading_noise_std_rad"])) if i else 0.0),
                "gnss_available": gnss_available,
                "gnss_x_m": x[i] + rng.normal(0.0, float(gnss["noise_std_m"]))
                if gnss_available
                else float("nan"),
                "gnss_y_m": y[i] + rng.normal(0.0, float(gnss["noise_std_m"]))
                if gnss_available
                else float("nan"),
            }
        )
    return truth, sensor_rows


def write_csv(path: Path, rows: list[dict[str, float | int]]) -> None:
    """Write rows using their first row as the CSV schema."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise ValueError("cannot write an empty scenario")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    with args.config.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    truth, sensor_rows = generate(config)
    output = Path(config["output"]["directory"])
    prefix = str(config["output"]["prefix"])
    write_csv(output / f"{prefix}_ground_truth.csv", truth)
    write_csv(output / f"{prefix}_sensors.csv", sensor_rows)
    metadata = {
        "scenario": config["scenario"],
        "sensors": config["sensors"],
        "ground_truth_file": f"{prefix}_ground_truth.csv",
        "sensor_file": f"{prefix}_sensors.csv",
        "notice": "Synthetic engineering fixture; not recorded vehicle data.",
    }
    output.mkdir(parents=True, exist_ok=True)
    with (output / f"{prefix}_metadata.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)
    print(f"Wrote {len(truth)} synchronized samples to {output}")


if __name__ == "__main__":
    main()
