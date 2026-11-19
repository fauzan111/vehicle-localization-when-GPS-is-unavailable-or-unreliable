# Multi-seed evaluation and sensor ablation

## Protocol

Ran 30 paired noise seeds (17–46) on the same 60-second, 20 Hz synthetic trajectory and the same two scheduled GNSS outages (15–30 s and 42–52 s). Each method saw the same generated sensor streams for a given seed. Intervals below are percentile-bootstrap 95% confidence intervals for the mean across these configured synthetic noise seeds (2,000 bootstrap resamples); they do not cover model, dataset, or real-world uncertainty.

## Outage-period position error

| Method | Mean RMSE | Bootstrap 95% CI for mean | Mean 95th-percentile error |
|---|---:|---:|---:|
| Wheel + gyro dead reckoning | 11.60 m | 11.32–11.89 m | 21.29 m |
| Camera relative-pose dead reckoning | 17.16 m | 12.25–23.24 m | 29.52 m |
| EKF: wheel + gyro + camera + intermittent GNSS | **0.68 m** | **0.55–0.82 m** | **1.41 m** |
| EKF without camera (wheel + gyro + intermittent GNSS) | 0.73 m | 0.59–0.88 m | 1.51 m |
| EKF without GNSS throughout the run | 14.06 m | 10.20–18.32 m | 25.28 m |

The paired mean outage-RMSE difference for full EKF minus no-camera EKF was **−0.053 m** (95% bootstrap CI **−0.130 to +0.017 m**). This interval includes zero, so this experiment does not show a clear camera benefit under the current sensor-noise assumptions. GNSS contributes much more: full EKF minus the no-GNSS-throughout-run ablation was **−13.38 m** (95% CI **−17.65 to −9.48 m**). The 30-run wheel/gyro-only EKF matches its dead-reckoning baseline, as expected when no aiding measurements are enabled.

## Recovery after GNSS reacquisition

For the full EKF, all **60 of 60** scheduled outages across the 30 seeds returned below 1 m position error for at least three consecutive samples. Mean time from reacquisition to that threshold was **0.014 s** (95% bootstrap CI **0.005–0.025 s**). At 20 Hz, this is a discrete-sample result: most runs were already below 1 m at the first reacquired sample. It should not be interpreted as a physical vehicle response-time guarantee.

## Interpretation and limits

- The filter estimates gyro-rate and wheel-speed biases as part of its state. The synthetic generator uses constant configured biases and Gaussian white measurement noise.
- Camera relative pose is generated from ground truth with independent noise. No image rendering, feature tracking, visual-odometry scale drift, tracking loss, or camera/IMU time offset is modeled.
- Camera-only integration is highly variable across seeds, and adding camera to intermittent GNSS gives no statistically clear improvement here. Better camera noise calibration, a motion profile that makes sensor complementarity observable, and realistic visual-odometry failure modes are needed before making a stronger claim.
- The no-GNSS ablation has no GNSS for the full run; its scheduled-window error is shown for convenience, but its all-run RMSE is also high (15.16 m). These remain synthetic results on one planar motion profile, not real-world validation.

Reproduce with `make sweep`. Machine-readable per-seed rows are in [`results/multi_seed.csv`](../results/multi_seed.csv), with aggregate metrics in [`results/multi_seed.yaml`](../results/multi_seed.yaml).
