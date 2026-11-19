"""Render a dependency-light SVG summary of the scene-0061 CAN replay."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml

from gps_denied_localization.baselines import read_csv
from gps_denied_localization.fusion import estimate


COLORS = {"reference pose": "#18212f", "wheel + IMU": "#2584d8", "GNSS aided EKF": "#e27a33"}


def polyline(x: np.ndarray, y: np.ndarray, box: tuple[float, float, float, float],
             bounds: tuple[float, float, float, float], color: str, width: float = 2.5) -> str:
    left, top, plot_width, plot_height = box
    xmin, xmax, ymin, ymax = bounds
    sx = plot_width / max(xmax - xmin, 1e-9)
    sy = plot_height / max(ymax - ymin, 1e-9)
    points = " ".join(
        f"{left + (float(xi) - xmin) * sx:.1f},{top + plot_height - (float(yi) - ymin) * sy:.1f}"
        for xi, yi in zip(x, y)
    )
    return f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round"/>'


def render(scene_dir: Path, output: Path) -> None:
    sensors, truth = read_csv(scene_dir / "sensors.csv"), read_csv(scene_dir / "ground_truth.csv")
    metadata = yaml.safe_load((scene_dir / "metadata.yaml").read_text(encoding="utf-8"))
    wheel, _, _ = estimate(sensors, metadata, use_camera=False, use_gnss=False)
    aided, _, _ = estimate(sensors, metadata, use_camera=False, use_gnss=True)
    time = sensors["time_s"]
    errors = {
        "wheel + IMU": np.hypot(wheel[:, 0] - truth["x_m"], wheel[:, 1] - truth["y_m"]),
        "GNSS aided EKF": np.hypot(aided[:, 0] - truth["x_m"], aided[:, 1] - truth["y_m"]),
    }
    extent = max(float(np.max(np.abs(truth["x_m"]))), float(np.max(np.abs(truth["y_m"]))), 1.0)
    margin = extent * 0.12
    position_bounds = (-extent - margin, extent + margin, -extent - margin, extent + margin)
    error_max = max(2.0, *(float(np.quantile(value, 0.99)) for value in errors.values()))
    svg: list[str] = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="760" viewBox="0 0 1200 760">',
        '<rect width="1200" height="760" fill="#f7f8fa"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#18212f}.muted{fill:#627084}.grid{stroke:#dce1e8;stroke-width:1}</style>',
        '<text x="48" y="48" font-size="28" font-weight="700">nuScenes CAN-only localization replay · scene-0061</text>',
        '<text x="48" y="77" font-size="15" class="muted">Recorded wheel speed + IMU · noisy GNSS synthesized from CAN pose · no camera input</text>',
        '<text x="48" y="124" font-size="19" font-weight="700">Local trajectory (m)</text>',
        '<text x="630" y="124" font-size="19" font-weight="700">Position error and simulated GNSS availability</text>',
    ]
    left_box = (90.0, 155.0, 430.0, 430.0)
    right_box = (665.0, 155.0, 480.0, 430.0)
    for box in (left_box, right_box):
        l, t, w, h = box
        for fraction in (0, 0.25, 0.5, 0.75, 1):
            x, y = l + fraction * w, t + fraction * h
            svg.append(f'<line x1="{x}" y1="{t}" x2="{x}" y2="{t+h}" class="grid"/>')
            svg.append(f'<line x1="{l}" y1="{y}" x2="{l+w}" y2="{y}" class="grid"/>')
        svg.append(f'<rect x="{l}" y="{t}" width="{w}" height="{h}" fill="none" stroke="#8792a2"/>')
    svg.extend([
        '<text x="260" y="610" text-anchor="middle" font-size="14" class="muted">local x (m)</text>',
        '<text x="55" y="370" text-anchor="middle" font-size="14" class="muted" transform="rotate(-90 55 370)">local y (m)</text>',
        '<text x="905" y="610" text-anchor="middle" font-size="14" class="muted">time (s)</text>',
        '<text x="635" y="370" text-anchor="middle" font-size="14" class="muted" transform="rotate(-90 635 370)">position error (m)</text>',
    ])
    x_bounds = (float(np.min(truth["x_m"])) - margin, float(np.max(truth["x_m"])) + margin)
    y_bounds = (float(np.min(truth["y_m"])) - margin, float(np.max(truth["y_m"])) + margin)
    # Shade the observed availability mask behind the error curves.
    unavailable = sensors["gnss_available"] == 0
    if np.any(unavailable):
        starts = np.flatnonzero(np.diff(np.r_[False, unavailable, False].astype(int)) == 1)
        stops = np.flatnonzero(np.diff(np.r_[False, unavailable, False].astype(int)) == -1)
        left, top, width, height = right_box
        for start, stop in zip(starts, stops):
            x0 = left + time[start] / time[-1] * width
            x1 = left + time[min(stop, len(time)-1)] / time[-1] * width
            svg.append(f'<rect x="{x0:.1f}" y="{top}" width="{max(1,x1-x0):.1f}" height="{height}" fill="#f3b44b" opacity="0.18"/>')
    for label, data, color in (
        ("reference pose", truth, COLORS["reference pose"]),
        ("wheel + IMU", wheel, COLORS["wheel + IMU"]),
        ("GNSS aided EKF", aided, COLORS["GNSS aided EKF"]),
    ):
        svg.append(polyline(data["x_m"] if isinstance(data, dict) else data[:, 0],
                            data["y_m"] if isinstance(data, dict) else data[:, 1],
                            left_box, (*x_bounds, *y_bounds), color))
    for name, error in errors.items():
        svg.append(polyline(time, error, right_box, (0.0, float(time[-1]), 0.0, error_max), COLORS[name]))
    for label, color, y in (("reference pose", COLORS["reference pose"], 630), ("wheel + IMU", COLORS["wheel + IMU"], 654), ("GNSS aided EKF", COLORS["GNSS aided EKF"], 678)):
        svg.append(f'<line x1="80" y1="{y}" x2="112" y2="{y}" stroke="{color}" stroke-width="4"/><text x="122" y="{y+5}" font-size="15">{label}</text>')
    svg.append('<rect x="495" y="644" width="16" height="16" fill="#f3b44b" opacity="0.45"/><text x="520" y="658" font-size="15">simulated GNSS outage</text>')
    svg.append(f'<text x="48" y="724" font-size="14" class="muted">{len(time)} samples · {time[-1]:.1f} s · GNSS fixes are synthetic · visualization is a diagnostic, not real-world validation</text>')
    svg.append('</svg>')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(svg), encoding="utf-8")
    print(f"Wrote {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-dir", type=Path, default=Path("data/generated/nuscenes_scene-0061"))
    parser.add_argument("--out", type=Path, default=Path("results/nuscenes_can_scene0061.svg"))
    args = parser.parse_args()
    render(args.scene_dir, args.out)


if __name__ == "__main__":
    main()
