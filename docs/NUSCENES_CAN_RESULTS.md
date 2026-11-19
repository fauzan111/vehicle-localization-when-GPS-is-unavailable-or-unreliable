# nuScenes mini CAN-only replay

## Scope

The adapter replays recorded CAN IMU and wheel-speed streams from all ten official mini scenes. CAN pose supplies the evaluation reference and is also used to synthesize noisy 1 Hz GNSS fixes with two scheduled outages per scene. No camera images or recorded GNSS receiver measurements were used. These numbers are a pipeline check, not evidence of real GNSS-denied performance or generalization.

Run from the project root:

```sh
PYTHONPATH=src python3 -m gps_denied_localization.nuscenes_can_sweep \
  --config configs/nuscenes_can.yaml --can-bus-dir can_bus
```

The command writes per-scene CSV and YAML under `results/nuscenes_can_mini.*`.

## Initial aggregate

| Method | Mean outage position RMSE across 10 scenes | Scene range | Mean outage p95 error |
|---|---:|---:|---:|
| Wheel + IMU + synthetic GNSS | 1.25 m | 0.56–2.40 m | 2.06 m |
| Wheel + IMU only | 0.62 m | 0.00–1.95 m | 0.92 m |

The aided filter recovered to stable <1 m error after 18 of 20 scheduled outage intervals, with a mean recovery time of 0.71 s among the per-scene recovery averages. The GNSS-aided method performed worse on this run: synthetic pose-derived GNSS noise, short scenes, filter assumptions, and only two outage windows per scene make these results diagnostic. The wheel/IMU baseline is also not a real independent ground-truth comparison: CAN pose is used for scoring, and the signals share the same vehicle and coordinate assumptions.

## Scope boundary and possible extension

The CAN-only demo is complete within its stated scope. A future camera extension could use synchronized `CAM_FRONT` visual odometry, but it is not required to reproduce the current results. Before making stronger performance claims, calibrate wheel radius/frame conventions, inspect per-scene sensor alignment, and use an independent reference with a broader recorded dataset.
