# nuScenes CAN replay and camera-data plan

## Selection

The downloaded CAN expansion has `pose`, `ms_imu`, and `zoe_veh_info` for all ten official nuScenes mini scenes. A CAN-only adapter and ten-scene replay are implemented; results and limitations are in [NUSCENES_CAN_RESULTS.md](NUSCENES_CAN_RESULTS.md). The mini camera payload (`v1.0-mini`) is not currently present, so visual odometry is not yet part of the replay. The CAN expansion supplies 100 Hz IMU measurements, 100 Hz four-wheel RPM, and 50 Hz ego pose; nuScenes cameras run at 12 Hz. See the [nuScenes data format](https://www.nuscenes.org/data-format) and the [official CAN bus README](https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/nuscenes/can_bus/README.md).

The CAN documentation calls its data experimental, so each signal and timestamp must be checked per scene. Some scenes have no CAN expansion and must be skipped. The official [CAN bus tutorial](https://www.nuscenes.org/tutorials/can_bus_tutorial.html) demonstrates converting wheel RPM to linear speed with a 0.305 m wheel-radius assumption and notes a small offset from vehicle-monitor speed.

## Sensor mapping

| Estimator stream | nuScenes source | Treatment |
|---|---|---|
| Camera | `CAM_FRONT` images and camera calibration | Run visual odometry on image pairs; output relative pose at image timestamps. |
| IMU | CAN `ms_imu` | Use timestamped angular velocity and acceleration; transform axes into the chosen vehicle frame. |
| Wheel speed | CAN `zoe_veh_info` FL/FR/RL/RR wheel RPM | Convert RPM to m/s using a documented radius, retain per-wheel channels for slip checks, and align by timestamp. |
| Reference pose | CAN `pose` position/orientation at 50 Hz | Evaluation only. Do not expose reference pose to the estimator except through a separately generated GNSS observation. |
| GNSS updates | Simulated from reference pose | Add configured position noise and hide fixes during controlled outages. Clearly label as simulated GNSS; this is not a recorded receiver stream. |

## First replay protocol

1. Completed: inspect the CAN streams and implement a 50 Hz pose-time grid for pose, IMU, and wheel speed. For `scene-0061`, inspection found 1,899 IMU samples, 1,904 wheel-info samples, and 938 pose samples over about 19.1 seconds.
2. Completed for CAN-only replay: interpolate IMU and wheel speed only within the recorded time span; use pose as evaluation reference, except when generating explicitly synthetic GNSS observations.
3. Normalize reference positions into a local frame and document the orientation convention and camera-to-vehicle extrinsics.
4. Generate synthetic GNSS fixes from reference pose, then replay 5-second outages at multiple offsets.
5. Completed partially: compare wheel/IMU dead reckoning and a GNSS-aided filter on all ten scenes. Next, download `v1.0-mini`, implement camera odometry, then rerun a camera/IMU/wheel ablation.

## Limits and access

nuScenes is available for non-commercial use after registration. Only the mini camera payload is needed for this prototype; the full trainval camera download is unnecessary. The CAN expansion is documented as experimental, so validate streams and timestamps before using each scene. The official [Oxford RobotCar download page currently says the dataset is withdrawn](https://robotcar-dataset.robots.ox.ac.uk/downloads/), so it is not the primary choice.

This replay will still not test real GNSS outages: GNSS observations are synthesized from the reference pose. It will test real recorded camera, IMU, and wheel-speed streams against a reference trajectory with a controlled GNSS-aiding simulation.
