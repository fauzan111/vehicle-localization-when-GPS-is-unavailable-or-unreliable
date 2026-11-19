# Data

`generated/` contains deterministic synthetic scenario outputs created by `make scenario`. It is git-ignored because each file can be regenerated from `configs/baseline.yaml`.

## Planned external replay

The CAN expansion is already available under this project's `can_bus/` directory, and the CAN-only adapter has replayed all ten official mini scenes. Results are in [NUSCENES_CAN_RESULTS.md](../docs/NUSCENES_CAN_RESULTS.md). To add visual odometry, download only the `v1.0-mini` camera payload from [nuScenes](https://www.nuscenes.org/nuscenes#download); the full trainval set is not needed. It is offered for non-commercial use.

Set `NUSCENES_ROOT` to the extracted mini dataset root and `CAN_BUS_DIR` to the directory containing `scene-0061_pose.json` and related files. In the current workspace, CAN files are under `can_bus/`. Keep downloaded data out of Git. Data source, sensor mapping, and replay limitations are described in [DATASET_PLAN.md](../docs/DATASET_PLAN.md).

Run the CAN sweep with `make nuscenes-can-sweep CAN_BUS_DIR=can_bus`. After downloading `v1.0-mini`, check channels and timestamps with `make inspect-nuscenes NUSCENES_ROOT=/path/to/nuscenes-mini CAN_BUS_DIR=can_bus` before implementing the camera stream.

For the external replay, camera, IMU, and wheel speed will be recorded streams; the evaluation reference pose comes from CAN `pose`. GNSS observations will be synthesized from that reference with configured noise and masked during outages, so results must be labeled as a simulated GNSS-aiding experiment rather than a test of recorded GNSS receiver dropouts. Mini has only 10 scenes, so it is suitable for an end-to-end prototype and initial per-scene comparisons, not broad generalization claims.
