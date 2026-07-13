import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.perception.simple_obstacle_detector import SimpleObstacleDetector
from src.planner.obstacle_config import build_obstacle_map, get_resolution_altitude, get_start_goal, load_obstacle_config


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test the simulated local obstacle detector without PX4 or Gazebo."
    )
    parser.add_argument(
        "--obstacle-config",
        type=Path,
        default=PROJECT_ROOT / "config" / "substation_obstacles.json",
        help="Obstacle config JSON. Default: config/substation_obstacles.json",
    )
    parser.add_argument(
        "--detection-range",
        type=float,
        default=4.0,
        help="Detection range in meters. Default: 4.0",
    )
    parser.add_argument(
        "--detection-fov",
        type=float,
        default=90.0,
        help="Forward field of view in degrees. Default: 90",
    )
    parser.add_argument(
        "--warning-distance",
        type=float,
        default=2.0,
        help="Warning risk threshold in meters. Default: 2.0",
    )
    parser.add_argument(
        "--danger-distance",
        type=float,
        default=1.0,
        help="Danger risk threshold in meters. Default: 1.0",
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=None,
        help="Flight altitude for height-aware inflated cells. Defaults to config altitude_m.",
    )
    return parser.parse_args()


def resolve_project_path(path):
    path = path.expanduser()
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def print_detection_result(name, north_m, east_m, yaw_deg, result):
    print(f"\n{name}")
    print(f"  local east/north: east={east_m:.1f} m, north={north_m:.1f} m")
    print(f"  yaw: {yaw_deg} deg")
    print(f"  detected obstacle count: {result['detected_obstacle_count']}")
    print(f"  risk level: {result['risk_level']}")

    nearest = result["nearest_obstacle"]
    if nearest is None:
        print("  nearest obstacle: none")
        print("  nearest distance: N/A")
        return

    print(
        "  nearest obstacle: "
        f"{nearest['obstacle_name']} "
        f"({nearest['obstacle_layer']}, {nearest['obstacle_type'] or 'unknown'}) "
        f"cell=({nearest['grid_x']}, {nearest['grid_y']})"
    )
    print(f"  nearest distance: {nearest['distance_m']:.2f} m")
    bearing = nearest["bearing_deg_relative"]
    print(f"  nearest bearing: {'N/A' if bearing is None else f'{bearing:.1f} deg'}")
    print("  first detections:")
    for obstacle in result["detected_obstacles"][:5]:
        bearing = obstacle["bearing_deg_relative"]
        print(
            "    - "
            f"{obstacle['obstacle_name']} "
            f"[{obstacle['obstacle_layer']}] "
            f"cell=({obstacle['grid_x']}, {obstacle['grid_y']}) "
            f"distance={obstacle['distance_m']:.2f} m "
            f"bearing={'N/A' if bearing is None else f'{bearing:.1f} deg'}"
        )


def main():
    args = parse_args()
    config_path = resolve_project_path(args.obstacle_config)
    config = load_obstacle_config(config_path)
    resolution_m, config_altitude_m = get_resolution_altitude(config)
    altitude_m = config_altitude_m if args.altitude is None else args.altitude
    start_cell, goal_cell = get_start_goal(config)
    obstacle_map = build_obstacle_map(
        config,
        flight_altitude_m=altitude_m,
        start_cell=start_cell,
        goal_cell=goal_cell,
    )

    detector = SimpleObstacleDetector(
        config,
        obstacle_map=obstacle_map,
        detection_range_m=args.detection_range,
        warning_distance_m=args.warning_distance,
        danger_distance_m=args.danger_distance,
        detection_fov_deg=args.detection_fov,
        use_inflated_cells=True,
        use_raw_cells=True,
        flight_altitude_m=altitude_m,
    )

    print(f"Obstacle config: {config_path}")
    print(f"Map: {config.get('map_name', config_path.stem)}")
    print(f"Resolution: {resolution_m:.2f} m/cell")
    print(f"Altitude: {altitude_m:.2f} m")
    print("Detector: simple_obstacle_detector")
    print(f"Detection range: {args.detection_range:.1f} m")
    print(f"Detection FOV: {args.detection_fov:.1f} deg")
    print(f"Warning distance: {args.warning_distance:.1f} m")
    print(f"Danger distance: {args.danger_distance:.1f} m")

    test_positions = [
        ("near start", 0.5, 0.5, 45.0),
        ("near transformer", 5.5, 4.0, 0.0),
        ("near central corridor", 9.5, 8.5, 0.0),
        ("near goal", 16.5, 16.5, 180.0),
    ]
    for name, north_m, east_m, yaw_deg in test_positions:
        result = detector.detect(
            local_north_m=north_m,
            local_east_m=east_m,
            yaw_deg=yaw_deg,
            altitude_m=altitude_m,
        )
        print_detection_result(name, north_m, east_m, yaw_deg, result)


if __name__ == "__main__":
    main()
