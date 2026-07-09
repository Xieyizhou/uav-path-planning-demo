"""CLI and configuration helpers for A* flight experiments.

This module keeps command-line parsing, default flight constants, obstacle-map
loading, and perception/replan configuration construction out of the runtime
orchestrator. It is used by `fly_astar_path.py` for both dry-run previews and
live PX4 SITL flights.
"""

import argparse
from datetime import datetime, timezone
from pathlib import Path

from src.planner.obstacle_config import (
    build_obstacle_map,
    get_resolution_altitude,
    get_start_goal,
    load_obstacle_config,
    validate_obstacle_config,
)
from src.logging.output_registry import get_previews_dir


PROJECT_ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = PROJECT_ROOT / "data" / "logs"
OUTPUT_ROOT = PROJECT_ROOT / "outputs"
PREVIEW_DIR = get_previews_dir("static_astar") / "as_preview"
DEFAULT_SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"
PLANNER_NAME = "astar_grid"
DEFAULT_MAP_NAME = "default_grid"

GRID_WIDTH = 10
GRID_HEIGHT = 10
GRID_START = (0, 0)
GRID_GOAL = (8, 8)
GRID_OBSTACLES = {
    (3, 0),
    (3, 1),
    (3, 2),
    (3, 3),
    (3, 5),
    (3, 6),
    (3, 7),
    (3, 8),
    (6, 2),
    (6, 3),
    (6, 4),
    (6, 5),
    (6, 6),
}

MAX_HORIZONTAL_SPEED_M_S = 0.8
MAX_VERTICAL_SPEED_M_S = 0.5
POSITION_GAIN = 0.6
REACHED_HORIZONTAL_ERROR_M = 0.4
REACHED_VERTICAL_ERROR_M = 0.4
WAYPOINT_TIMEOUT_S = 30
WAYPOINT_TIMEOUT_MODE = "auto"
RETURN_SPEED_SCALE = 0.7
TURN_SETTLE_S = 1.0
MIN_RISK_SPEED_M_S = 0.3


def _list_cells(cells):
    return [[x, y] for x, y in cells]


def parse_args():
    """Parse the standalone flight-script CLI.

    Returns:
        An argparse namespace used to configure planning, MAVSDK connection,
        perception risk response, local replanning, and dry-run preview mode.
    """
    parser = argparse.ArgumentParser(
        description="Plan an A* grid path and optionally fly it in PX4 SITL."
    )
    parser.add_argument(
        "--system-address",
        default=DEFAULT_SYSTEM_ADDRESS,
        help=f"MAVSDK system address. Default: {DEFAULT_SYSTEM_ADDRESS}",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Plan and save a preview without connecting to PX4 or flying.",
    )
    parser.add_argument(
        "--allow-diagonal",
        action="store_true",
        help="Allow diagonal moves in the A* grid planner.",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=1.0,
        help="Meters per grid cell. Default: 1.0",
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=None,
        help="Target altitude above takeoff in meters. Overrides obstacle config altitude_m.",
    )
    parser.add_argument(
        "--obstacle-config",
        type=Path,
        help="Optional JSON obstacle config, for example config/substation_obstacles.json.",
    )
    parser.add_argument(
        "--return-home",
        action="store_true",
        help=(
            "After reaching the A* goal, fly the reversed A* waypoint path "
            "back to the start before landing. This does not use PX4 RTL."
        ),
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=MAX_HORIZONTAL_SPEED_M_S,
        help=f"Maximum horizontal A* speed in m/s. Default: {MAX_HORIZONTAL_SPEED_M_S}",
    )
    parser.add_argument(
        "--waypoint-acceptance",
        type=float,
        default=REACHED_HORIZONTAL_ERROR_M,
        help=f"Horizontal waypoint acceptance radius in meters. Default: {REACHED_HORIZONTAL_ERROR_M}",
    )
    parser.add_argument(
        "--waypoint-timeout",
        default="auto",
        help=(
            "Per-waypoint timeout in seconds, or 'auto' to estimate from waypoint "
            "distance, speed, return speed scale, and perception risk action. Default: auto"
        ),
    )
    parser.add_argument(
        "--turn-settle",
        type=float,
        default=TURN_SETTLE_S,
        help=f"Seconds to hover after reaching each waypoint. Default: {TURN_SETTLE_S}",
    )
    parser.add_argument(
        "--return-speed-scale",
        type=float,
        default=RETURN_SPEED_SCALE,
        help=f"Return route speed multiplier. Default: {RETURN_SPEED_SCALE}",
    )
    parser.add_argument(
        "--enable-perception",
        action="store_true",
        help="Enable simulated local obstacle detection in the telemetry log.",
    )
    parser.add_argument(
        "--detection-range",
        type=float,
        default=4.0,
        help="Simulated perception detection range in meters. Default: 4.0",
    )
    parser.add_argument(
        "--detection-fov",
        type=float,
        default=90.0,
        help="Simulated forward perception field of view in degrees. Default: 90",
    )
    parser.add_argument(
        "--warning-distance",
        type=float,
        default=2.0,
        help="Distance in meters for warning-level perception risk. Default: 2.0",
    )
    parser.add_argument(
        "--danger-distance",
        type=float,
        default=1.0,
        help="Distance in meters for danger-level perception risk. Default: 1.0",
    )
    parser.add_argument(
        "--risk-action",
        choices=["log_only", "slow_down", "stop_and_land"],
        default="log_only",
        help="Optional behavior for perception risk. Default: log_only",
    )
    parser.add_argument(
        "--min-risk-speed",
        type=float,
        default=MIN_RISK_SPEED_M_S,
        help=(
            "Minimum horizontal commanded speed in m/s when --risk-action slow_down "
            f"is reducing speed. Default: {MIN_RISK_SPEED_M_S}"
        ),
    )
    parser.add_argument(
        "--perception-use-inflated",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use inflated planning cells for simulated perception. Default: true",
    )
    parser.add_argument(
        "--perception-use-raw",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use raw physical obstacle cells for simulated perception. Default: true",
    )
    parser.add_argument(
        "--enable-local-replan",
        action="store_true",
        help=(
            "Attempt local A* replans when perception risk reaches the configured "
            "threshold."
        ),
    )
    parser.add_argument(
        "--replan-mode",
        choices=["log_only", "active"],
        default="log_only",
        help="Local replanning mode. Default: log_only",
    )
    parser.add_argument(
        "--replan-risk-level",
        choices=["detected", "warning", "danger"],
        default="danger",
        help="Minimum perception risk level that triggers a local replan attempt. Default: danger",
    )
    parser.add_argument(
        "--replan-cooldown",
        type=float,
        default=5.0,
        help="Minimum seconds between local replan attempts. Default: 5.0",
    )
    parser.add_argument(
        "--dynamic-obstacle-inflation",
        type=int,
        default=1,
        help="Grid-cell inflation applied to perception-derived dynamic obstacles. Default: 1",
    )
    parser.add_argument(
        "--max-replans",
        type=int,
        default=5,
        help="Maximum local replan attempts per flight. Default: 5",
    )
    return parser.parse_args()


def make_log_path():
    """Return the timestamped CSV path for the next A* telemetry log."""
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    return LOG_DIR / f"astar_{timestamp}.csv"


def default_planner_config(resolution_m, altitude_m):
    """Build the small built-in grid map used when no obstacle config is passed."""
    if altitude_m is None:
        altitude_m = 2.5
    return {
        "map_name": DEFAULT_MAP_NAME,
        "width": GRID_WIDTH,
        "height": GRID_HEIGHT,
        "start": GRID_START,
        "goal": GRID_GOAL,
        "obstacles": GRID_OBSTACLES,
        "raw_obstacle_cells": GRID_OBSTACLES,
        "raw_blocking_cells": GRID_OBSTACLES,
        "inflated_blocking_cells": GRID_OBSTACLES,
        "blocking_obstacle_cells": GRID_OBSTACLES,
        "inflated_obstacle_cells": GRID_OBSTACLES,
        "raw_obstacle_cell_to_name": {},
        "inflated_obstacle_cell_to_name": {},
        "resolution_m": resolution_m,
        "altitude_m": altitude_m,
        "vertical_safety_margin_m": 0.0,
        "horizontal_inflation_cells": 0,
        "blocking_obstacle_names": [],
        "nonblocking_obstacle_names": [],
        "raw_obstacle_cell_count": len(GRID_OBSTACLES),
        "raw_blocking_cell_count": len(GRID_OBSTACLES),
        "obstacle_cell_count": len(GRID_OBSTACLES),
        "inflated_obstacle_cell_count": len(GRID_OBSTACLES),
        "obstacle_config_path": None,
        "obstacle_config": None,
        "obstacle_map": None,
        "validation_warnings": [],
    }


def load_planner_config(args):
    """Load and normalize planner settings from CLI arguments.

    Args:
        args: Parsed CLI arguments from `parse_args`.

    Returns:
        A dictionary containing grid dimensions, start/goal cells, height-aware
        obstacle cells, local waypoint conversion settings, and validation
        warnings. When `--obstacle-config` is provided, this uses the JSON
        obstacle map; otherwise it falls back to the built-in toy grid.

    Side effects:
        Prints the selected map, obstacle counts, and validation warnings to
        make preview and flight logs self-describing.
    """
    if args.obstacle_config is None:
        print("Using built-in default A* grid.")
        return default_planner_config(args.resolution, args.altitude)

    config_path = args.obstacle_config
    if not config_path.is_absolute():
        config_path = PROJECT_ROOT / config_path

    config = load_obstacle_config(config_path)
    start, goal = get_start_goal(config)
    resolution_m, config_altitude_m = get_resolution_altitude(config)
    altitude_m = args.altitude if args.altitude is not None else config_altitude_m
    obstacle_map = build_obstacle_map(
        config,
        flight_altitude_m=altitude_m,
        start_cell=start,
        goal_cell=goal,
    )
    validation_warnings = validate_obstacle_config(
        config,
        args.allow_diagonal,
        flight_altitude_m=altitude_m,
    )
    obstacles = obstacle_map["inflated_blocking_cells"]

    planner_config = {
        "map_name": config.get("map_name", config_path.stem),
        "width": int(config["width"]),
        "height": int(config["height"]),
        "start": start,
        "goal": goal,
        "obstacles": obstacles,
        "raw_obstacle_cells": obstacle_map["raw_obstacle_cells"],
        "raw_blocking_cells": obstacle_map["raw_blocking_cells"],
        "inflated_blocking_cells": obstacle_map["inflated_blocking_cells"],
        "blocking_obstacle_cells": obstacle_map["raw_blocking_cells"],
        "inflated_obstacle_cells": obstacle_map["inflated_blocking_cells"],
        "raw_obstacle_cell_to_name": obstacle_map["raw_obstacle_cell_to_name"],
        "inflated_obstacle_cell_to_name": obstacle_map["inflated_obstacle_cell_to_name"],
        "resolution_m": resolution_m,
        "altitude_m": obstacle_map["altitude_m"],
        "vertical_safety_margin_m": obstacle_map["vertical_safety_margin_m"],
        "horizontal_inflation_cells": obstacle_map["horizontal_inflation_cells"],
        "blocking_obstacle_names": obstacle_map["blocking_obstacle_names"],
        "nonblocking_obstacle_names": obstacle_map["nonblocking_obstacle_names"],
        "raw_obstacle_cell_count": obstacle_map["raw_obstacle_cell_count"],
        "raw_blocking_cell_count": obstacle_map["raw_blocking_cell_count"],
        "obstacle_cell_count": obstacle_map["obstacle_cell_count"],
        "inflated_obstacle_cell_count": obstacle_map["inflated_obstacle_cell_count"],
        "obstacle_config_path": config_path,
        "obstacle_config": config,
        "obstacle_map": obstacle_map,
        "validation_warnings": validation_warnings,
    }

    print(f"Using obstacle config: {display_path(config_path)}")
    print(f"Map name: {planner_config['map_name']}")
    print(f"Start cell: {planner_config['start']}")
    print(f"Goal cell: {planner_config['goal']}")
    print(f"Resolution: {resolution_m} m/cell")
    print("Height-aware planning:")
    print(f"  flight altitude: {planner_config['altitude_m']} m")
    print(f"  vertical safety margin: {planner_config['vertical_safety_margin_m']} m")
    print(f"  horizontal inflation cells: {planner_config['horizontal_inflation_cells']}")
    print(f"  blocking obstacles: {planner_config['blocking_obstacle_names']}")
    print(f"  ignored/nonblocking obstacles: {planner_config['nonblocking_obstacle_names']}")
    print(f"Raw physical footprint cells: {planner_config['raw_obstacle_cell_count']}")
    print(f"Height-blocking raw footprint cells: {planner_config['raw_blocking_cell_count']}")
    print(f"Inflated blocking obstacle cells: {planner_config['inflated_obstacle_cell_count']}")
    print(f"Raw physical footprint cells: {_list_cells(sorted(planner_config['raw_obstacle_cells']))}")
    print(f"Inflated planning obstacle cells: {_list_cells(sorted(planner_config['inflated_blocking_cells']))}")
    if validation_warnings:
        print("Obstacle config validation warnings:")
        for warning in validation_warnings:
            print(f"  WARNING: {warning}")
    else:
        print("Obstacle config validation: OK")
    return planner_config


def display_path(path):
    try:
        return path.resolve().relative_to(PROJECT_ROOT)
    except ValueError:
        return path.resolve()


def build_perception_config(args):
    """Return perception settings consumed by the detector, logger, and flight loop."""
    return {
        "enabled": args.enable_perception,
        "detector_name": "simple_obstacle_detector",
        "detection_range_m": args.detection_range,
        "detection_fov_deg": args.detection_fov,
        "warning_distance_m": args.warning_distance,
        "danger_distance_m": args.danger_distance,
        "risk_action": args.risk_action,
        "use_raw_cells": args.perception_use_raw,
        "use_inflated_cells": args.perception_use_inflated,
    }


def print_perception_summary(perception_config):
    """Print the active perception configuration before preview or flight."""
    print("\nPerception:")
    print(f"  enabled: {str(perception_config['enabled']).lower()}")
    print(f"  detector: {perception_config['detector_name']}")
    print(f"  detection range: {perception_config['detection_range_m']} m")
    print(f"  detection FOV: {perception_config['detection_fov_deg']} deg")
    print(f"  warning distance: {perception_config['warning_distance_m']} m")
    print(f"  danger distance: {perception_config['danger_distance_m']} m")
    print(f"  risk action: {perception_config['risk_action']}")
    print(f"  uses raw cells: {str(perception_config['use_raw_cells']).lower()}")
    print(f"  uses inflated cells: {str(perception_config['use_inflated_cells']).lower()}")


def build_replan_config(args, planner_config):
    """Return local-replan settings derived from CLI arguments and map metadata.

    The returned dictionary is intentionally plain data so `fly_astar_path.py`
    can log it, test trigger thresholds, and call A* without reaching back into
    argparse or obstacle-config internals.
    """
    if args.replan_cooldown < 0:
        raise ValueError("--replan-cooldown must be non-negative")
    if args.dynamic_obstacle_inflation < 0:
        raise ValueError("--dynamic-obstacle-inflation must be non-negative")
    if args.max_replans < 0:
        raise ValueError("--max-replans must be non-negative")

    enabled = bool(args.enable_local_replan)
    if enabled and not args.enable_perception:
        raise ValueError("--enable-local-replan requires --enable-perception")
    if enabled and planner_config.get("obstacle_map") is None:
        raise ValueError("--enable-local-replan requires --obstacle-config")

    return {
        "enabled": enabled,
        "mode": args.replan_mode,
        "risk_level": args.replan_risk_level,
        "cooldown_s": args.replan_cooldown,
        "dynamic_obstacle_inflation": args.dynamic_obstacle_inflation,
        "max_replans": args.max_replans,
        "width": planner_config["width"],
        "height": planner_config["height"],
        "resolution_m": planner_config["resolution_m"],
        "altitude_m": planner_config["altitude_m"],
        "goal_cell": planner_config["goal"],
        "static_obstacles": set(planner_config["inflated_blocking_cells"]),
        "allow_diagonal": args.allow_diagonal,
    }


def print_replan_summary(replan_config):
    """Print the active local-replan configuration before preview or flight."""
    print("\nLocal replanning:")
    print(f"  enabled: {str(replan_config['enabled']).lower()}")
    print(f"  mode: {replan_config['mode']}")
    print(f"  trigger risk level: {replan_config['risk_level']}")
    print(f"  cooldown: {replan_config['cooldown_s']} s")
    print(f"  dynamic obstacle inflation: {replan_config['dynamic_obstacle_inflation']} cell(s)")
    print(f"  max replans: {replan_config['max_replans']}")
    print(f"  active waypoint replacement: {str(replan_config['mode'] == 'active').lower()}")
