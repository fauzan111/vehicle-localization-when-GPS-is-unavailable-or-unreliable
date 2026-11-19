"""Convert one nuScenes CAN scene into synchronized localization replay files."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def load_messages(directory: Path, scene: str, stream: str) -> list[dict[str, Any]]:
    path = directory / f"{scene}_{stream}.json"
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open(encoding="utf-8") as file:
        messages = json.load(file)
    if not isinstance(messages, list) or not messages:
        raise ValueError(f"Expected non-empty message list: {path}")
    timestamps = [int(row["utime"]) for row in messages]
    if np.any(np.diff(timestamps) <= 0):
        raise ValueError(f"Timestamps must increase strictly: {path}")
    return messages


def quaternion_yaw(quaternion: list[float]) -> float:
    """Convert nuScenes [w, x, y, z] orientation to planar yaw."""
    w, x, y, z = (float(value) for value in quaternion)
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def convert_scene(
    can_bus_dir: Path, scene: str, config: dict[str, Any]
) -> tuple[list[dict[str, float | int]], list[dict[str, float | int]], dict[str, Any]]:
    pose = load_messages(can_bus_dir, scene, "pose")
    imu = load_messages(can_bus_dir, scene, "ms_imu")
    wheels = load_messages(can_bus_dir, scene, "zoe_veh_info")

    origin_us = int(pose[0]["utime"])

    def relative_seconds(rows: list[dict[str, Any]]) -> np.ndarray:
        return np.asarray(
            [(int(row["utime"]) - origin_us) / 1e6 for row in rows], dtype=float
        )

    pose_time = relative_seconds(pose)
    imu_time = relative_seconds(imu)
    wheel_time = relative_seconds(wheels)
    common_start = max(pose_time[0], imu_time[0], wheel_time[0])
    common_end = min(pose_time[-1], imu_time[-1], wheel_time[-1])
    keep = (pose_time >= common_start) & (pose_time <= common_end)
    pose_time = pose_time[keep]
    pose = [row for row, selected in zip(pose, keep) if selected]
    if len(pose_time) < 3:
        raise ValueError("The three CAN streams have insufficient common overlap")
    time = pose_time - pose_time[0]

    positions_world = np.asarray([row["pos"][:2] for row in pose], dtype=float)
    positions_world -= positions_world[0]
    absolute_heading = np.unwrap([quaternion_yaw(row["orientation"]) for row in pose])
    initial_heading = absolute_heading[0]
    cosine, sine = np.cos(initial_heading), np.sin(initial_heading)
    positions = np.column_stack(
        (
            cosine * positions_world[:, 0] + sine * positions_world[:, 1],
            -sine * positions_world[:, 0] + cosine * positions_world[:, 1],
        )
    )
    heading = absolute_heading - initial_heading
    truth = [
        {
            "time_s": time[i],
            "x_m": positions[i, 0],
            "y_m": positions[i, 1],
            "heading_rad": heading[i],
        }
        for i in range(len(time))
    ]

    radius = float(config["streams"]["wheel_speed"]["wheel_radius_m"])
    wheel_keys = ("FL_wheel_speed", "FR_wheel_speed", "RL_wheel_speed", "RR_wheel_speed")
    wheel_rpm = np.asarray(
        [[float(row[key]) for key in wheel_keys] for row in wheels], dtype=float
    ).mean(axis=1)
    wheel_speed = wheel_rpm * (2.0 * np.pi * radius / 60.0)
    gyro_z = np.asarray(
        [float(row["rotation_rate"][2]) for row in imu], dtype=float
    )
    aligned_wheel = np.interp(pose_time, wheel_time, wheel_speed)
    aligned_gyro = np.interp(pose_time, imu_time, gyro_z)
    pose_heading_rate = np.diff(heading) / np.diff(time)
    gyro_mid = np.interp(
        (time[1:] + time[:-1]) / 2.0, pose_time, aligned_gyro
    )
    pose_forward_speed = (
        np.diff(positions[:, 0]) * np.cos(heading[:-1])
        + np.diff(positions[:, 1]) * np.sin(heading[:-1])
    ) / np.diff(time)

    gnss_config = config["streams"]["gnss"]
    noise_std = float(gnss_config["noise_std_m"])
    rng = np.random.default_rng(int(config.get("seed", 17)))
    outage_windows = [
        (float(start), float(end)) for start, end in gnss_config["outages_s"]
    ]
    gnss_available = np.asarray(
        [int(not any(start <= t < end for start, end in outage_windows)) for t in time],
        dtype=int,
    )
    gnss_xy = positions + rng.normal(0.0, noise_std, size=positions.shape)

    rate = float(gnss_config["update_rate_hz"])
    update_indices: set[int] = set()
    update_times = np.arange(rate, time[-1], rate)
    for update_time in update_times:
        nearest = int(np.argmin(np.abs(time - update_time)))
        if gnss_available[nearest]:
            update_indices.add(nearest)
    gnss_update = np.asarray([int(i in update_indices) for i in range(len(time))])

    sensor_rows = [
        {
            "time_s": time[i],
            "wheel_speed_mps": aligned_wheel[i],
            "imu_gyro_z_rps": aligned_gyro[i],
            "gnss_available": gnss_available[i],
            "gnss_update": gnss_update[i],
            "gnss_x_m": gnss_xy[i, 0] if gnss_available[i] else float("nan"),
            "gnss_y_m": gnss_xy[i, 1] if gnss_available[i] else float("nan"),
        }
        for i in range(len(time))
    ]

    metadata = {
        "scenario": {
            "name": f"nuscenes_can_{scene}",
            "seed": int(config.get("seed", 17)),
            "duration_s": float(time[-1]),
            "sample_rate_hz": float(1.0 / np.median(np.diff(time))),
        },
        "sensors": {
            "camera_delta_pose": config["filter_noise"]["camera"],
            "wheel_speed": {
                "noise_std_mps": float(config["filter_noise"]["wheel_speed_std_mps"])
            },
            "imu_gyro_z": {
                "noise_std_rps": float(config["filter_noise"]["gyro_rate_std_rps"])
            },
            "gnss_position": {"noise_std_m": noise_std},
        },
        "notice": (
            "Recorded nuScenes CAN IMU, wheel speed, and reference pose. GNSS is "
            "synthetic noisy reference pose with configured outage masks. Camera is not used."
        ),
        "scene": scene,
        "alignment_diagnostics": {
            "yaw_rate_imu_correlation": float(np.corrcoef(pose_heading_rate, gyro_mid)[0, 1]),
            "mean_wheel_speed_mps": float(np.mean(aligned_wheel)),
            "mean_pose_forward_speed_mps": float(np.mean(pose_forward_speed)),
            "mean_wheel_minus_pose_speed_mps": float(
                np.mean(aligned_wheel[1:] - pose_forward_speed)
            ),
        },
    }
    return truth, sensor_rows, metadata


def write_csv(path: Path, rows: list[dict[str, float | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--can-bus-dir", type=Path, required=True)
    parser.add_argument("--scene")
    args = parser.parse_args()
    with args.config.open(encoding="utf-8") as file:
        config = yaml.safe_load(file)
    scene = args.scene or config["dataset"]["scene"]
    truth, sensors, metadata = convert_scene(args.can_bus_dir, scene, config)
    output = Path("data/generated") / f"nuscenes_{scene}"
    write_csv(output / "ground_truth.csv", truth)
    write_csv(output / "sensors.csv", sensors)
    with (output / "metadata.yaml").open("w", encoding="utf-8") as file:
        yaml.safe_dump(metadata, file, sort_keys=False)
    print(
        f"Wrote {len(truth)} samples for {scene} ({metadata['scenario']['duration_s']:.2f} s); "
        f"{sum(row['gnss_update'] for row in sensors)} synthetic GNSS fixes"
    )


if __name__ == "__main__":
    main()
