"""Run simple odometry baselines against a generated ground-truth trajectory."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def read_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"no samples in {path}")
    return {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}


def integrate_baselines(sensors: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Integrate wheel+gyro and camera-relative motion from a shared origin."""
    time = sensors["time_s"]
    count = len(time)
    outputs = {
        name: {key: np.zeros(count, dtype=float) for key in ("x_m", "y_m", "heading_rad")}
        for name in ("wheel_imu", "camera")
    }
    for i in range(1, count):
        dt = time[i] - time[i - 1]
        if dt <= 0:
            raise ValueError("sensor timestamps must increase strictly")

        wheel = outputs["wheel_imu"]
        wheel["heading_rad"][i] = (
            wheel["heading_rad"][i - 1] + sensors["imu_gyro_z_rps"][i - 1] * dt
        )
        wheel_distance = sensors["wheel_speed_mps"][i - 1] * dt
        wheel["x_m"][i] = wheel["x_m"][i - 1] + wheel_distance * np.cos(
            wheel["heading_rad"][i - 1]
        )
        wheel["y_m"][i] = wheel["y_m"][i - 1] + wheel_distance * np.sin(
            wheel["heading_rad"][i - 1]
        )

        camera = outputs["camera"]
        theta = camera["heading_rad"][i - 1]
        dx_body = sensors["camera_dx_body_m"][i]
        dy_body = sensors["camera_dy_body_m"][i]
        camera["x_m"][i] = camera["x_m"][i - 1] + np.cos(theta) * dx_body - np.sin(theta) * dy_body
        camera["y_m"][i] = camera["y_m"][i - 1] + np.sin(theta) * dx_body + np.cos(theta) * dy_body
        camera["heading_rad"][i] = theta + sensors["camera_dheading_rad"][i]
    return {name: np.column_stack(tuple(values.values())) for name, values in outputs.items()}


def summarize(
    estimates: dict[str, np.ndarray], truth: dict[str, np.ndarray], sensors: dict[str, np.ndarray]
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    outages = sensors["gnss_available"] == 0
    for name, estimate in estimates.items():
        dx = estimate[:, 0] - truth["x_m"]
        dy = estimate[:, 1] - truth["y_m"]
        position_error = np.hypot(dx, dy)
        heading_error = np.arctan2(
            np.sin(estimate[:, 2] - truth["heading_rad"]),
            np.cos(estimate[:, 2] - truth["heading_rad"]),
        )
        result[name] = {
            "position_rmse_m_all": float(np.sqrt(np.mean(position_error**2))),
            "position_p95_m_all": float(np.quantile(position_error, 0.95)),
            "heading_rmse_rad_all": float(np.sqrt(np.mean(heading_error**2))),
            "position_rmse_m_gnss_outage": float(np.sqrt(np.mean(position_error[outages] ** 2)))
            if np.any(outages)
            else None,
            "position_p95_m_gnss_outage": float(np.quantile(position_error[outages], 0.95))
            if np.any(outages)
            else None,
        }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensors", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    sensors, truth = read_csv(args.sensors), read_csv(args.ground_truth)
    estimates = integrate_baselines(sensors)
    metrics = summarize(estimates, truth, sensors)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            {
                "notice": "Synthetic single-seed baselines; not real-world validation.",
                "sample_count": len(sensors["time_s"]),
                "gnss_outage_sample_count": int(np.sum(sensors["gnss_available"] == 0)),
                "metrics": metrics,
            },
            stream,
            sort_keys=False,
        )
    for name, values in metrics.items():
        print(
            f"{name}: outage position RMSE {values['position_rmse_m_gnss_outage']:.3f} m, "
            f"all-time heading RMSE {values['heading_rmse_rad_all']:.4f} rad"
        )


if __name__ == "__main__":
    main()
