.PHONY: setup scenario baseline fusion evaluate sweep inspect-nuscenes nuscenes-can nuscenes-fusion nuscenes-can-sweep nuscenes-demo clean-generated
CAN_BUS_DIR ?= $(NUSCENES_ROOT)/can_bus

setup:
	uv sync

scenario:
	uv run python -m gps_denied_localization.scenario --config configs/baseline.yaml

baseline: scenario
	uv run python -m gps_denied_localization.baselines --sensors data/generated/baseline_sensors.csv \
	  --ground-truth data/generated/baseline_ground_truth.csv --out data/generated/baseline_metrics.yaml

fusion: scenario
	uv run python -m gps_denied_localization.fusion --sensors data/generated/baseline_sensors.csv \
	  --ground-truth data/generated/baseline_ground_truth.csv \
	  --metadata data/generated/baseline_metadata.yaml --out data/generated/fusion

evaluate: baseline fusion

sweep:
	uv run python -m gps_denied_localization.experiments --config configs/baseline.yaml \
	  --seeds 30 --start-seed 17 --out results/multi_seed

inspect-nuscenes:
	uv run python scripts/inspect_nuscenes_can.py --dataroot "$(NUSCENES_ROOT)" \
	  --can-bus-dir "$(CAN_BUS_DIR)" --scene scene-0061

nuscenes-can:
	PYTHONPATH=src uv run python -m gps_denied_localization.nuscenes_can \
	  --config configs/nuscenes_can.yaml --can-bus-dir "$(CAN_BUS_DIR)"

nuscenes-fusion: nuscenes-can
	uv run python -m gps_denied_localization.fusion \
	  --sensors data/generated/nuscenes_scene-0061/sensors.csv \
	  --ground-truth data/generated/nuscenes_scene-0061/ground_truth.csv \
	  --metadata data/generated/nuscenes_scene-0061/metadata.yaml \
	  --out data/generated/nuscenes_scene-0061/fusion --no-camera

nuscenes-can-sweep:
	PYTHONPATH=src uv run python -m gps_denied_localization.nuscenes_can_sweep \
	  --config configs/nuscenes_can.yaml --can-bus-dir "$(CAN_BUS_DIR)"

nuscenes-demo: nuscenes-can nuscenes-can-sweep
	PYTHONPATH=src uv run python scripts/render_nuscenes_can_demo.py

clean-generated:
	rm -f data/generated/baseline_*.csv data/generated/baseline_metadata.yaml \
	  data/generated/baseline_metrics.yaml data/generated/fusion.csv data/generated/fusion.yaml
