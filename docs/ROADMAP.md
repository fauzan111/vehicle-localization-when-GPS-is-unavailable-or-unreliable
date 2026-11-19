# Roadmap

## Milestone 1 — Reproducible scenario and sensor log (complete)

- Define the common time base, ground-truth pose, and sensor record format.
- Generate a deterministic planar trajectory with noisy wheel speed, gyro yaw rate, camera-relative motion, and configurable GNSS outages.
- Save ground truth and sensor streams separately so estimators cannot accidentally consume ground truth.
- **Exit:** one command generates the same schema and scenario for a fixed seed; metadata records seed, rates, noise, and outage windows. Implemented in `scenario.py`.

## Milestone 2 — Baselines (initial single-seed comparison complete)

- Implement wheel-speed + gyro dead reckoning.
- Implement camera-odometry-only integration.
- Add GNSS-enabled evaluation as an upper-bound reference.
- **Exit:** report drift against ground truth for each baseline with the same input scenario. Initial camera-only and wheel+gyro results are in [BASELINE_RESULTS.md](BASELINE_RESULTS.md); extend to repeated seeds and GNSS upper-bound before treating this milestone as complete.

## Milestone 3 — Sensor fusion (initial EKF implemented)

- Implement a planar error-state EKF (or clearly justified alternative) for camera, IMU, wheel-speed, and intermittent GNSS.
- Handle missing and delayed GNSS updates explicitly; expose estimator covariance.
- **Exit:** ablation shows the contribution of each sensor and numerical behavior remains stable through dropouts.
- Current prototype uses a planar `[x, y, yaw, gyro bias, wheel-speed bias]` EKF and handles camera pose as a relative measurement over consecutive states. Initial results are in [FUSION_RESULTS.md](FUSION_RESULTS.md). The 30-seed sensor ablation is in [MULTI_SEED_RESULTS.md](MULTI_SEED_RESULTS.md); camera benefit is not clear under current assumptions, so noise calibration and harder motion cases remain open.

## Milestone 4 — Evaluation and stress tests (30-seed baseline complete)

- Evaluate straight, turn, and lane-change motion across 5/15/30-second GNSS outages and multiple noise seeds. The current 30-seed run covers one smoothly varying planar path and 15/10-second outages; it does not yet meet the full scenario matrix.
- Report position RMSE, 95th-percentile error, heading RMSE, outage drift rate, reacquisition recovery time, and confidence intervals across runs.
- Add failure cases: low visual texture, camera blur, wheel slip, gyro bias, and GNSS multipath/outliers.
- **Exit:** reproducible report and plots, with limitations and failure modes stated.

## Milestone 5 — nuScenes CAN-only replay (complete within revised scope)

- The CAN adapter converts pose, IMU, and wheel-speed records for all ten official mini scene names; `make nuscenes-can-sweep CAN_BUS_DIR=can_bus` replays the ten-scene comparison. Results are in [NUSCENES_CAN_RESULTS.md](NUSCENES_CAN_RESULTS.md).
- Camera visual odometry is excluded from this version because the `v1.0-mini` images are unavailable locally; the CAN-only scope is explicitly documented.
- Treat GNSS outages as synthetic masks over noisy reference-pose observations; do not call these recorded GNSS outages.
- **Exit for this version:** convert and align all ten mini-scene CAN streams, run the two estimator variants, save per-scene metrics, and provide reproducible setup and result notes. This does not satisfy the broader camera-based localization objective.

## Milestone 6 — CAN-only portfolio demo (complete)

- Produce a short replay showing ground truth, estimate, GNSS availability, and error over time.
- Document setup, commands, configuration, results, and known gaps.
- **Exit:** `make nuscenes-demo CAN_BUS_DIR=can_bus` regenerates the replay outputs and chart from the local CAN expansion. The report states the experiment's scope and limitations.

## Decisions to lock early

- Start planar (2D) to keep the state and failure analysis understandable.
- Keep scenario generation independent from estimator code and never pass ground truth to an estimator.
- Keep synthetic results clearly labeled; do not describe them as real-world validation.
- Future extension if camera data becomes available: add synchronized image-based odometry and an independent reference before making broader localization claims.
