"""Run paired multi-seed sensor-fusion and ablation experiments."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from gps_denied_localization.baselines import integrate_baselines, summarize
from gps_denied_localization.fusion import estimate, outage_analysis
from gps_denied_localization.scenario import generate

METHODS = {
    "wheel_imu_dead_reckoning": ("wheel_imu", True, False),
    "camera_dead_reckoning": ("camera", True, False),
    "ekf_full": ("ekf", True, True),
    "ekf_no_camera": ("ekf", False, True),
    "ekf_no_gnss": ("ekf", True, False),
    "ekf_wheel_imu_only": ("ekf", False, False),
}
METRICS = (
    "position_rmse_m_all",
    "position_p95_m_all",
    "heading_rmse_rad_all",
    "position_rmse_m_gnss_outage",
    "position_p95_m_gnss_outage",
)
RECOVERY_METRICS = ("recovery_time_mean_s", "outages_recovered_count")


def rows_to_arrays(rows: list[dict[str, float | int]]) -> dict[str, np.ndarray]:
    return {
        key: np.asarray([float(row[key]) for row in rows], dtype=float)
        for key in rows[0]
    }


def bootstrap_mean_interval(values: np.ndarray, rng: np.random.Generator) -> list[float]:
    samples = rng.choice(values, size=(2000, len(values)), replace=True).mean(axis=1)
    return [float(np.quantile(samples, 0.025)), float(np.quantile(samples, 0.975))]


def run(config: dict[str, Any], seeds: int, start_seed: int) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for seed in range(start_seed, start_seed + seeds):
        run_config = {**config, "scenario": {**config["scenario"], "seed": seed}}
        truth_rows, sensor_rows = generate(run_config)
        truth, sensors = rows_to_arrays(truth_rows), rows_to_arrays(sensor_rows)
        estimates = integrate_baselines(sensors)
        metadata = {"sensors": run_config["sensors"]}
        ekf_cache: dict[tuple[bool, bool], np.ndarray] = {}

        for name, (kind, use_camera, use_gnss) in METHODS.items():
            if kind in ("wheel_imu", "camera"):
                estimate_states = estimates[kind]
            else:
                key = (use_camera, use_gnss)
                if key not in ekf_cache:
                    ekf_states, _, _ = estimate(
                        sensors,
                        metadata,
                        use_camera=use_camera,
                        use_gnss=use_gnss,
                    )
                    ekf_cache[key] = ekf_states[:, :3]
                estimate_states = ekf_cache[key]
            scores = summarize({name: estimate_states}, truth, sensors)[name]
            if name == "ekf_full":
                outage_rows = outage_analysis(sensors, estimate_states, truth)
                recovery_times = [
                    row["recovery_time_to_stable_1m_s"]
                    for row in outage_rows
                    if row["recovery_time_to_stable_1m_s"] is not None
                ]
                scores["recovery_time_mean_s"] = (
                    float(np.mean(recovery_times)) if recovery_times else None
                )
                scores["outages_recovered_count"] = len(recovery_times)
            records.append({"seed": seed, "method": name, **scores})
    return records


def summarize_runs(records: list[dict[str, Any]], seed_count: int) -> dict[str, Any]:
    rng = np.random.default_rng(42_671)
    summary: dict[str, Any] = {"seed_count": seed_count, "methods": {}}
    for method in METHODS:
        method_rows = [row for row in records if row["method"] == method]
        method_summary: dict[str, Any] = {}
        for metric in METRICS:
            values = np.asarray([row[metric] for row in method_rows], dtype=float)
            method_summary[metric] = {
                "mean": float(np.mean(values)),
                "std_between_seeds": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
                "bootstrap_95pct_ci_of_mean": bootstrap_mean_interval(values, rng),
            }
        summary["methods"][method] = method_summary
        if method == "ekf_full":
            for metric in RECOVERY_METRICS:
                values = np.asarray([row[metric] for row in method_rows], dtype=float)
                if metric == "recovery_time_mean_s":
                    method_summary[metric] = {
                        "mean": float(np.mean(values)),
                        "std_between_seeds": float(np.std(values, ddof=1)),
                        "bootstrap_95pct_ci_of_mean": bootstrap_mean_interval(values, rng),
                    }
                else:
                    method_summary[metric] = {
                        "per_seed_mean": float(np.mean(values)),
                        "total_outages": int(np.sum(values)),
                        "planned_outages": int(seed_count * 2),
                    }
    by_pair = {
        (row["seed"], row["method"]): row["position_rmse_m_gnss_outage"]
        for row in records
    }
    comparisons = {}
    for ablation in ("ekf_no_camera", "ekf_no_gnss"):
        differences = np.asarray(
            [
                by_pair[(seed, "ekf_full")] - by_pair[(seed, ablation)]
                for seed in range(
                    min(row["seed"] for row in records),
                    max(row["seed"] for row in records) + 1,
                )
            ],
            dtype=float,
        )
        comparisons[f"ekf_full_minus_{ablation}_outage_rmse_m"] = {
            "mean_paired_difference": float(np.mean(differences)),
            "bootstrap_95pct_ci_of_mean": bootstrap_mean_interval(differences, rng),
            "interpretation": "Negative values favor the full sensor set.",
        }
    summary["paired_comparisons"] = comparisons
    return summary


def write_records(path: Path, records: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["seed", "method", *METRICS, *RECOVERY_METRICS]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--seeds", type=int, default=30)
    parser.add_argument("--start-seed", type=int, default=17)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.seeds < 2:
        parser.error("--seeds must be at least 2 for an across-seed interval")
    with args.config.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    records = run(config, args.seeds, args.start_seed)
    summary = summarize_runs(records, args.seeds)
    summary["notice"] = (
        "Synthetic paired-seed experiment. Bootstrap intervals quantify uncertainty "
        "in the mean over configured noise seeds only; they are not real-world confidence bounds."
    )
    summary["seed_range_inclusive"] = [args.start_seed, args.start_seed + args.seeds - 1]
    write_records(args.out.with_suffix(".csv"), records)
    yaml_path = args.out.with_suffix(".yaml")
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    with yaml_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(summary, stream, sort_keys=False)
    for method, metrics in summary["methods"].items():
        value = metrics["position_rmse_m_gnss_outage"]
        lo, hi = value["bootstrap_95pct_ci_of_mean"]
        print(f"{method}: outage RMSE {value['mean']:.2f} m (bootstrap 95% CI {lo:.2f}–{hi:.2f})")


if __name__ == "__main__":
    main()
