"""Inspect required nuScenes CAN messages before building a scene replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

MESSAGES = {
    "ms_imu": ("rotation_rate", "linear_accel"),
    "pose": ("utime", "pos", "orientation"),
    "zoe_veh_info": (
        "FL_wheel_speed",
        "FR_wheel_speed",
        "RL_wheel_speed",
        "RR_wheel_speed",
    ),
}


def load_messages(can_directory: Path, scene: str, message: str) -> list[dict[str, Any]]:
    path = can_directory / f"{scene}_{message}.json"
    if not path.is_file():
        raise FileNotFoundError(f"Missing CAN message file: {path}")
    with path.open(encoding="utf-8") as stream:
        messages = json.load(stream)
    if not isinstance(messages, list) or not messages:
        raise ValueError(f"Expected a non-empty JSON list in {path}")
    return messages


def inspect(can_directory: Path, scene: str) -> dict[str, Any]:
    report: dict[str, Any] = {"scene": scene, "streams": {}}
    for message, required_fields in MESSAGES.items():
        rows = load_messages(can_directory, scene, message)
        missing = [field for field in required_fields if field not in rows[0]]
        if missing:
            raise ValueError(f"{message} is missing required fields: {missing}")
        if "utime" not in rows[0]:
            raise ValueError(f"{message} has no utime timestamp")
        timestamps = np.asarray([int(row["utime"]) for row in rows], dtype=np.int64)
        if np.any(np.diff(timestamps) <= 0):
            raise ValueError(f"{message} timestamps are not strictly increasing")
        intervals_s = np.diff(timestamps) / 1e6
        median_period = float(np.median(intervals_s))
        report["streams"][message] = {
            "sample_count": len(rows),
            "duration_s": float((timestamps[-1] - timestamps[0]) / 1e6),
            "median_rate_hz": float(1.0 / median_period),
            "timestamp_min_us": int(timestamps[0]),
            "timestamp_max_us": int(timestamps[-1]),
            "first_record_fields": sorted(rows[0]),
        }
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataroot", type=Path, required=True)
    parser.add_argument("--can-bus-dir", type=Path)
    parser.add_argument("--scene", default="scene-0001")
    args = parser.parse_args()
    if len(args.scene) != 10 or not args.scene.startswith("scene-") or not args.scene[-4:].isdigit():
        parser.error("--scene must use the nuScenes form scene-0001")
    can_directory = args.can_bus_dir or args.dataroot / "can_bus"
    if (can_directory / "can_bus").is_dir():
        can_directory /= "can_bus"
    report = inspect(can_directory, args.scene)
    image_directory = args.dataroot / "samples" / "CAM_FRONT"
    report["cam_front_directory_present"] = image_directory.is_dir()
    print(json.dumps(report, indent=2))
    if not report["cam_front_directory_present"]:
        print("Warning: samples/CAM_FRONT is missing; download the full camera data for image replay.")


if __name__ == "__main__":
    main()
