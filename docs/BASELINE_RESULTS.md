# Preliminary synthetic baselines

Generated with `configs/baseline.yaml` (seed 17, 20 Hz, 60 s, 1,201 samples, 500 samples without GNSS). The trajectory and all sensor noise are synthetic. Camera-relative motion is generated directly from simulated ground truth with configured noise; it is not produced by an image-based visual-odometry algorithm. These are pipeline sanity measurements, not evidence of real sensor performance.

| Baseline | Position RMSE during GNSS outages | 95th percentile during outages | Heading RMSE, full run |
|---|---:|---:|---:|
| Wheel speed + gyro integration | 12.88 m | 23.77 m | 0.0755 rad |
| Camera relative-pose integration | 8.13 m | 13.62 m | 0.0495 rad |

Camera-relative increments are indexed by the destination frame. The values above were regenerated after fixing the baseline integration to consume increments at that timestamp. The gap is expected from the fixture: camera translation has lower configured incremental noise and no visual tracking failures, while wheel speed and gyro have persistent biases. The metrics use one random seed, so there is no confidence interval. The initial fused estimator and reacquisition analysis are now in [FUSION_RESULTS.md](FUSION_RESULTS.md).

Reproduce with `make baseline` after installing the project dependencies. In this restricted environment, `uv sync` could not reach PyPI; the generation and baseline commands were run with the available system Python, NumPy, and PyYAML.
