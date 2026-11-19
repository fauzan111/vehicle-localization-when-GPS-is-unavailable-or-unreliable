# Initial fused localization result

## Setup

This is one deterministic 60-second synthetic run (seed 17, 20 Hz, 1,201 samples) with GNSS outages from 15–30 s and 42–52 s. The planar EKF propagates with wheel speed and gyro rate, estimates gyro-rate and wheel-speed biases, updates with camera relative pose, and applies global GNSS position fixes when available. Its state is `[x, y, yaw, gyro bias, wheel-speed bias]`. A joint previous/current state covariance is used for camera relative updates. A 30-seed follow-up is documented in [MULTI_SEED_RESULTS.md](MULTI_SEED_RESULTS.md).

## Results

| Metric | Initial EKF result |
|---|---:|
| Position RMSE, full run | 1.48 m |
| Position RMSE, GNSS unavailable | 2.19 m |
| 95th percentile position error, GNSS unavailable | 3.79 m |
| Heading RMSE, full run | 0.0261 rad |
| Position RMSE while GNSS is available | 0.58 m |

| GNSS outage | Start error | Maximum error during outage | Time to return below 1 m for 3 consecutive samples after reacquisition |
|---|---:|---:|---:|
| 15–30 s | 0.19 m | 3.99 m | 0.65 s |
| 42–52 s | 0.75 m | 4.15 m | 1.05 s |

The recovery threshold and three-sample condition are an initial reporting convention, not a vehicle requirement. Full per-sample estimates, uncertainty, and update flags are emitted to `data/generated/fusion.csv`; summary metrics are in `data/generated/fusion.yaml`.

## Limits

- The camera signal is a simulated relative-pose measurement derived from ground truth with configured noise. No images are rendered and no visual-odometry algorithm runs yet.
- Sensor noise values are assumed inputs, not calibrated from hardware or a dataset.
- This is a single seed and a planar model. It does not establish generalization, statistical confidence, 3D performance, or safe vehicle use.
- The estimator does not yet model wheel slip, GNSS outliers/multipath, camera tracking loss, or asynchronous timestamps.

Reproduce with `make evaluate` after installing dependencies. In the current restricted environment, `uv sync` could not download packages from PyPI; equivalent module commands ran with the available system Python, NumPy, and PyYAML.
