"""Run the planar sensor-fusion EKF over timestamped generated sensor data."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import yaml

from gps_denied_localization.baselines import read_csv, summarize
from gps_denied_localization.ekf import PlanarLocalizationEKF


def estimate(
    sensors: dict[str, np.ndarray],
    metadata: dict,
    use_camera: bool = True,
    use_gnss: bool = True,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return filtered states, covariance diagonals, and GNSS update flags."""
    noise = metadata["sensors"]
    ekf = PlanarLocalizationEKF(
        camera_translation_std_m=float(
            noise["camera_delta_pose"]["translation_noise_std_m"]
        ),
        camera_heading_std_rad=float(
            noise["camera_delta_pose"]["heading_noise_std_rad"]
        ),
        gnss_position_std_m=float(noise["gnss_position"]["noise_std_m"]),
        wheel_speed_std_mps=float(noise["wheel_speed"]["noise_std_mps"]),
        gyro_rate_std_rps=float(noise["imu_gyro_z"]["noise_std_rps"]),
    )
    count = len(sensors["time_s"])
    states = np.zeros((count, 5), dtype=float)
    covariance_diagonal = np.zeros((count, 5), dtype=float)
    gnss_update = np.zeros(count, dtype=int)
    states[0] = ekf.state
    covariance_diagonal[0] = np.diag(ekf.covariance)

    for i in range(1, count):
        dt = sensors["time_s"][i] - sensors["time_s"][i - 1]
        ekf.predict(
            sensors["wheel_speed_mps"][i - 1],
            sensors["imu_gyro_z_rps"][i - 1],
            dt,
        )
        if use_camera:
            ekf.update_camera(
                sensors["camera_dx_body_m"][i],
                sensors["camera_dy_body_m"][i],
                sensors["camera_dheading_rad"][i],
            )
        gnss_updates = sensors.get("gnss_update", sensors["gnss_available"])
        if (
            use_gnss
            and sensors["gnss_available"][i] > 0
            and gnss_updates[i] > 0
        ):
            ekf.update_gnss(sensors["gnss_x_m"][i], sensors["gnss_y_m"][i])
            gnss_update[i] = 1
        states[i] = ekf.state
        covariance_diagonal[i] = np.diag(ekf.covariance)
    return states, covariance_diagonal, gnss_update


def write_estimates(
    path: Path,
    time: np.ndarray,
    states: np.ndarray,
    covariance_diagonal: np.ndarray,
    gnss_update: np.ndarray,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "time_s",
        "x_m",
        "y_m",
        "heading_rad",
        "gyro_bias_rps",
        "wheel_speed_bias_mps",
        "x_std_m",
        "y_std_m",
        "heading_std_rad",
        "gnss_update",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for i, timestamp in enumerate(time):
            writer.writerow(
                {
                    "time_s": timestamp,
                    "x_m": states[i, 0],
                    "y_m": states[i, 1],
                    "heading_rad": states[i, 2],
                    "gyro_bias_rps": states[i, 3],
                    "wheel_speed_bias_mps": states[i, 4],
                    "x_std_m": np.sqrt(covariance_diagonal[i, 0]),
                    "y_std_m": np.sqrt(covariance_diagonal[i, 1]),
                    "heading_std_rad": np.sqrt(covariance_diagonal[i, 2]),
                    "gnss_update": gnss_update[i],
                }
            )


def outage_analysis(
    sensors: dict[str, np.ndarray], states: np.ndarray, truth: dict[str, np.ndarray]
) -> list[dict[str, float | int | None]]:
    """Summarize drift during each outage and time to stable 1 m recovery."""
    time = sensors["time_s"]
    error = np.hypot(states[:, 0] - truth["x_m"], states[:, 1] - truth["y_m"])
    unavailable = sensors["gnss_available"] == 0
    edges = np.diff(np.r_[False, unavailable, False].astype(int))
    starts = np.flatnonzero(edges == 1)
    stops = np.flatnonzero(edges == -1)
    reports: list[dict[str, float | int | None]] = []
    for start, stop in zip(starts, stops):
        reacquisition = int(stop) if stop < len(time) else None
        recovery_time: float | None = None
        if reacquisition is not None:
            for index in range(reacquisition, max(reacquisition, len(time) - 2)):
                if np.all(error[index : index + 3] <= 1.0):
                    recovery_time = float(time[index] - time[reacquisition])
                    break
        end_index = int(stop) - 1
        reports.append(
            {
                "start_s": float(time[start]),
                "end_s": float(time[stop]) if reacquisition is not None else float(time[-1]),
                "duration_s": float(time[end_index] - time[start]),
                "start_error_m": float(error[start]),
                "max_error_m": float(np.max(error[start : stop])),
                "end_error_m": float(error[end_index]),
                "recovery_time_to_stable_1m_s": recovery_time,
            }
        )
    return reports


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensors", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--no-camera", action="store_true")
    args = parser.parse_args()
    with args.metadata.open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    sensors, truth = read_csv(args.sensors), read_csv(args.ground_truth)
    states, covariance_diagonal, gnss_update = estimate(
        sensors, metadata, use_camera=not args.no_camera
    )
    estimates = {"ekf_fusion": states[:, :3]}
    metrics = summarize(estimates, truth, sensors)["ekf_fusion"]
    gnss_available = sensors["gnss_available"] > 0
    dx_available = states[gnss_available, 0] - truth["x_m"][gnss_available]
    dy_available = states[gnss_available, 1] - truth["y_m"][gnss_available]
    metrics["gnss_available_position_rmse_m"] = float(
        np.sqrt(np.mean(dx_available**2 + dy_available**2))
    )
    write_estimates(
        args.out.with_suffix(".csv"),
        sensors["time_s"],
        states,
        covariance_diagonal,
        gnss_update,
    )
    report_path = args.out.with_suffix(".yaml")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    notice = "GNSS measurements are synthetic noisy reference pose with configured outage masks."
    if not args.no_camera:
        notice += " Camera relative pose is idealized noise, not image-based visual odometry."
    else:
        notice += " Camera updates are disabled for this replay."
    with report_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            {
                "notice": notice,
                "camera_updates_enabled": not args.no_camera,
                "sample_count": len(sensors["time_s"]),
                "gnss_outage_sample_count": int(np.sum(sensors["gnss_available"] == 0)),
                "metrics": metrics,
                "outage_events": outage_analysis(sensors, states, truth),
            },
            stream,
            sort_keys=False,
        )
    print(
        f"EKF: outage position RMSE {metrics['position_rmse_m_gnss_outage']:.3f} m, "
        f"GNSS-available position RMSE {metrics['gnss_available_position_rmse_m']:.3f} m"
    )


if __name__ == "__main__":
    main()

