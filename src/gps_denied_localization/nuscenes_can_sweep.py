"""Evaluate CAN-only localization across all ten official nuScenes mini scenes."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from gps_denied_localization.baselines import summarize
from gps_denied_localization.fusion import estimate, outage_analysis
from gps_denied_localization.nuscenes_can import convert_scene

MINI_SCENES = (
    "scene-0061",
    "scene-0553",
    "scene-0655",
    "scene-0757",
    "scene-0796",
    "scene-1077",
    "scene-1094",
    "scene-1100",
    "scene-0103",
    "scene-0916",
)


def arrays(rows: list[dict[str, float | int]]) -> dict[str, np.ndarray]:
    return {
        key: np.asarray([float(row[key]) for row in rows], dtype=float)
        for key in rows[0]
    }


def run(can_bus_dir: Path, config: dict[str, Any]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for scene in MINI_SCENES:
        scene_config = {
            **config,
            "seed": int(config.get("seed", 17)) + int(scene[-4:]),
        }
        truth_rows, sensor_rows, metadata = convert_scene(can_bus_dir, scene, scene_config)
        truth, sensors = arrays(truth_rows), arrays(sensor_rows)
        for method, use_gnss in (("wheel_imu_gnss_aided", True), ("wheel_imu_only", False)):
            states, _, _ = estimate(
                sensors,
                metadata,
                use_camera=False,
                use_gnss=use_gnss,
            )
            scores = summarize({method: states[:, :3]}, truth, sensors)[method]
            record: dict[str, Any] = {
                "scene": scene,
                "duration_s": metadata["scenario"]["duration_s"],
                "sample_count": len(truth_rows),
                "method": method,
                **scores,
            }
            if use_gnss:
                outages = outage_analysis(sensors, states[:, :3], truth)
                recovered = [
                    item["recovery_time_to_stable_1m_s"]
                    for item in outages
                    if item["recovery_time_to_stable_1m_s"] is not None
                ]
                record["recovery_events"] = len(recovered)
                record["recovery_time_mean_s"] = (
                    float(np.mean(recovered)) if recovered else None
                )
            records.append(record)
    return records


def aggregate(records: list[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for method in ("wheel_imu_gnss_aided", "wheel_imu_only"):
        rows = [row for row in records if row["method"] == method]
        keys = (
            "position_rmse_m_all",
            "position_rmse_m_gnss_outage",
            "position_p95_m_gnss_outage",
            "heading_rmse_rad_all",
        )
        result[method] = {
            metric: {
                "mean_across_scenes": float(np.mean([row[metric] for row in rows])),
                "median_across_scenes": float(np.median([row[metric] for row in rows])),
                "min_across_scenes": float(np.min([row[metric] for row in rows])),
                "max_across_scenes": float(np.max([row[metric] for row in rows])),
            }
            for metric in keys
        }
        result[method]["scene_count"] = len(rows)
    aided = [row for row in records if row["method"] == "wheel_imu_gnss_aided"]
    recovered = [row["recovery_events"] for row in aided]
    recovery_means = [
        row["recovery_time_mean_s"]
        for row in aided
        if row["recovery_time_mean_s"] is not None
    ]
    result["wheel_imu_gnss_aided"]["recovery"] = {
        "recovered_outage_count": int(sum(recovered)),
        "scheduled_outage_count": len(aided) * 2,
        "mean_recovery_time_per_scene_s": float(np.mean(recovery_means))
        if recovery_means
        else None,
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--can-bus-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("results/nuscenes_can_mini"))
    args = parser.parse_args()
    with args.config.open(encoding="utf-8") as file:
        config = yaml.safe_load(file)
    records = run(args.can_bus_dir, config)
    report = {
        "notice": (
            "Recorded nuScenes CAN wheel speed and IMU, scored against CAN pose. GNSS "
            "fixes are synthetic noisy pose observations. No camera images were used. "
            "Scene range is descriptive for this fixed 10-scene mini split."
        ),
        "scene_names": list(MINI_SCENES),
        "aggregate": aggregate(records),
        "per_scene": records,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    csv_path = args.out.with_suffix(".csv")
    fields = [
        "scene",
        "duration_s",
        "sample_count",
        "method",
        "position_rmse_m_all",
        "position_p95_m_all",
        "position_rmse_m_gnss_outage",
        "position_p95_m_gnss_outage",
        "heading_rmse_rad_all",
        "recovery_events",
        "recovery_time_mean_s",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)
    yaml_path = args.out.with_suffix(".yaml")
    with yaml_path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(report, file, sort_keys=False)
    for method, metrics in report["aggregate"].items():
        outage_rmse = metrics["position_rmse_m_gnss_outage"]
        print(
            f"{method}: outage RMSE mean {outage_rmse['mean_across_scenes']:.2f} m "
            f"(scene range {outage_rmse['min_across_scenes']:.2f}–"
            f"{outage_rmse['max_across_scenes']:.2f} m)"
        )


if __name__ == "__main__":
    main()
