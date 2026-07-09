"""Lightweight check for the simulated perception state layer.

This script exercises the map-based detector and structured perception state
without PX4, Gazebo, or MAVSDK. It is useful for quickly verifying risk levels,
field-of-view filtering, and suggested actions after perception changes.
"""

import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.perception.perception_state import build_perception_state
from src.perception.simple_obstacle_detector import SimpleObstacleDetector


class LocalPosition:
    """Minimal stand-in for MAVSDK local position telemetry."""

    def __init__(self, north_m, east_m, down_m=-1.5):
        self.north_m = north_m
        self.east_m = east_m
        self.down_m = down_m


class Attitude:
    """Minimal stand-in for MAVSDK Euler attitude telemetry."""

    def __init__(self, yaw_deg):
        self.yaw_deg = yaw_deg


def load_config():
    """Load the substation obstacle config used by the flight experiments."""
    config_path = PROJECT_ROOT / "config" / "substation_obstacles.json"
    with config_path.open() as config_file:
        return json.load(config_file)


def state_for(detector, perception_config, north_m, east_m, yaw_deg):
    """Build a structured perception state for one synthetic pose."""
    position = LocalPosition(north_m=north_m, east_m=east_m)
    attitude = Attitude(yaw_deg=yaw_deg)
    detection = detector.detect(
        local_north_m=position.north_m,
        local_east_m=position.east_m,
        yaw_deg=attitude.yaw_deg,
        altitude_m=1.5,
    )
    return build_perception_state(
        perception_config,
        detection=detection,
        position=position,
        attitude=attitude,
    )


def assert_state(label, state, risk_level, in_range, in_fov, suggested_action):
    """Assert the expected risk classification for a named synthetic case."""
    assert state["enabled"] is True, label
    assert state["risk_level"] == risk_level, (label, state)
    assert state["in_detection_range"] is in_range, (label, state)
    assert state["in_fov"] is in_fov, (label, state)
    assert state["suggested_action"] == suggested_action, (label, state)
    assert "position" in state and "heading" in state, (label, state)


def main():
    """Run clear, detected, warning, danger, and outside-FOV perception checks."""
    obstacle_config = load_config()
    detector = SimpleObstacleDetector(
        obstacle_config,
        detection_range_m=4.0,
        warning_distance_m=2.0,
        danger_distance_m=1.0,
        detection_fov_deg=90.0,
        use_raw_cells=True,
        use_inflated_cells=False,
    )
    perception_config = {
        "enabled": True,
        "risk_action": "slow_down",
    }

    clear = state_for(detector, perception_config, north_m=0.5, east_m=0.5, yaw_deg=90.0)
    detected = state_for(detector, perception_config, north_m=4.5, east_m=2.5, yaw_deg=90.0)
    warning = state_for(detector, perception_config, north_m=4.5, east_m=4.0, yaw_deg=90.0)
    danger = state_for(detector, perception_config, north_m=4.5, east_m=4.9, yaw_deg=90.0)
    outside_fov = state_for(detector, perception_config, north_m=4.5, east_m=2.5, yaw_deg=-90.0)

    assert_state("clear", clear, "clear", False, True, "none")
    assert_state("detected", detected, "detected", True, True, "none")
    assert_state("warning", warning, "warning", True, True, "slow_down")
    assert_state("danger", danger, "danger", True, True, "slow_down")
    assert_state("outside_fov", outside_fov, "clear", True, False, "none")

    for label, state in [
        ("clear", clear),
        ("detected", detected),
        ("warning", warning),
        ("danger", danger),
        ("outside_fov", outside_fov),
    ]:
        print(
            f"{label}: risk={state['risk_level']} "
            f"value={state['risk_value']} "
            f"closest={state['closest_obstacle_name'] or 'none'} "
            f"distance={state['closest_obstacle_distance_m']} "
            f"in_range={state['in_detection_range']} "
            f"in_fov={state['in_fov']} "
            f"action={state['suggested_action']}"
        )


if __name__ == "__main__":
    main()
