"""Simulated perception state construction and detector setup."""

from src.perception.perception_state import build_perception_state
from src.perception.simple_obstacle_detector import SimpleObstacleDetector

from src.flight.flight_state import value_or_blank


class DangerObstacleDetected(Exception):
    """Raised when the configured risk action requires an immediate landing."""


def current_perception_detection(
    perception_config,
    detector,
    position,
    attitude,
    timestamp_utc=None,
    elapsed_s=None,
    replan_config=None,
):
    enabled = bool(perception_config.get("enabled"))
    if not enabled or detector is None or position is None:
        return build_perception_state(
            perception_config,
            detection=None,
            position=position,
            attitude=attitude,
            timestamp_utc=timestamp_utc,
            elapsed_s=elapsed_s,
            replan_config=replan_config,
        )
    yaw_deg = value_or_blank(attitude, "yaw_deg")
    altitude_m = -position.down_m if hasattr(position, "down_m") else ""
    detection = detector.detect(
        local_north_m=position.north_m,
        local_east_m=position.east_m,
        yaw_deg=yaw_deg,
        altitude_m=altitude_m,
    )
    return build_perception_state(
        perception_config,
        detection=detection,
        position=position,
        attitude=attitude,
        timestamp_utc=timestamp_utc,
        elapsed_s=elapsed_s,
        replan_config=replan_config,
    )


def build_perception_detector(args, planner_config):
    if not args.enable_perception:
        return None
    if planner_config.get("obstacle_config") is None or planner_config.get("obstacle_map") is None:
        raise ValueError("--enable-perception requires --obstacle-config")
    return SimpleObstacleDetector(
        planner_config["obstacle_config"],
        obstacle_map=planner_config["obstacle_map"],
        detection_range_m=args.detection_range,
        warning_distance_m=args.warning_distance,
        danger_distance_m=args.danger_distance,
        detection_fov_deg=args.detection_fov,
        use_inflated_cells=args.perception_use_inflated,
        use_raw_cells=args.perception_use_raw,
        flight_altitude_m=planner_config["altitude_m"],
    )

