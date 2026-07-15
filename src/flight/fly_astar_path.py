"""Plan and fly A* waypoint routes in PX4 SITL.

This is the runtime orchestrator for the UAV simulation pipeline. It loads a
grid obstacle map, plans an A* route, optionally enables simulated perception
and local replanning, commands PX4 through MAVSDK Offboard velocity setpoints,
and streams telemetry to CSV for later experiment analysis.

The module intentionally keeps flight execution, perception response, and
active route replacement in one place for now. Refactor only in a dedicated
behavior-preserving task.
"""

import asyncio
import contextlib
import csv
import json
import os
import sys
from datetime import datetime, timezone
from math import floor, sqrt
from pathlib import Path

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.planner.astar_grid import (
    astar,
    cell_to_local_waypoint,
    grid_path_to_local_waypoints,
    simplify_grid_path,
)
from src.planner.obstacle_config import inflate_cells
from src.perception.perception_state import (
    build_perception_state,
    risk_reaches_threshold,
)
from src.perception.simple_obstacle_detector import SimpleObstacleDetector
from src.flight.flight_config import (
    LOG_DIR,
    CONNECTION_TIMEOUT_S,
    LANDING_TIMEOUT_S,
    LOGGER_SHUTDOWN_TIMEOUT_S,
    MAX_HORIZONTAL_SPEED_M_S,
    MAX_VERTICAL_SPEED_M_S,
    MIN_RISK_SPEED_M_S,
    OUTPUT_ROOT,
    PLANNER_NAME,
    POSITION_READY_TIMEOUT_S,
    POSITION_GAIN,
    PREVIEW_DIR,
    REACHED_HORIZONTAL_ERROR_M,
    REACHED_VERTICAL_ERROR_M,
    RETURN_SPEED_SCALE,
    TURN_SETTLE_S,
    TELEMETRY_TIMEOUT_S,
    WAYPOINT_TIMEOUT_MODE,
    build_perception_config,
    build_replan_config,
    display_path,
    load_planner_config,
    make_log_path,
    parse_args,
    print_perception_summary,
    print_replan_summary,
    validate_planner_safety,
    validate_runtime_args,
)
from src.flight.async_runtime import cancel_tasks, run_with_bounded_shutdown
from src.logging.flight_logger import (
    TELEMETRY_CSV_HEADER,
    build_telemetry_log_row,
    perception_csv_values,
    replan_csv_values,
)

class DangerObstacleDetected(Exception):
    """Raised when the configured risk action requires an immediate landing."""

    pass


def status_path_for_log(log_path):
    return log_path.with_suffix(".status.json")


def write_run_status(log_path, status, phase, message="", landing_confirmed=None):
    """Atomically persist the latest machine-readable flight outcome."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    status_path = status_path_for_log(log_path)
    payload = {
        "run_id": log_path.stem.replace("astar_", "as_", 1),
        "status": status,
        "phase": phase,
        "message": message,
        "landing_confirmed": landing_confirmed,
        "log_path": str(display_path(log_path)),
        "updated_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    temporary_path = status_path.with_suffix(status_path.suffix + ".tmp")
    temporary_path.write_text(json.dumps(payload, indent=2) + "\n")
    temporary_path.replace(status_path)
    return status_path

# Keep matplotlib cache files inside the project for dry-run preview plotting.
MPLCONFIG_DIR = OUTPUT_ROOT / ".matplotlib_cache"
MPLCONFIG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIG_DIR))

from src.flight.preview_writer import save_preview


def plan_path(planner_config, allow_diagonal):
    """Run A* and convert the grid path into local NED waypoints.

    Args:
        planner_config: Normalized map and obstacle settings from
            `flight_config.load_planner_config`.
        allow_diagonal: Whether A* may use diagonal grid moves.

    Returns:
        A tuple of `(grid_path, simplified_path, waypoints)`, where waypoints
        are local NED targets consumed by the MAVSDK flight loop.
    """
    grid_path = astar(
        start=planner_config["start"],
        goal=planner_config["goal"],
        obstacles=planner_config["obstacles"],
        width=planner_config["width"],
        height=planner_config["height"],
        allow_diagonal=allow_diagonal,
    )
    simplified_path = simplify_grid_path(grid_path)
    waypoints = grid_path_to_local_waypoints(
        simplified_path,
        resolution_m=planner_config["resolution_m"],
        altitude_m=planner_config["altitude_m"],
    )
    return grid_path, simplified_path, waypoints


def print_plan(grid_path, simplified_path, waypoints):
    print(f"Original A* path length: {len(grid_path)}")
    print(f"Simplified waypoint count: {len(waypoints)}")
    print(f"Grid path: {grid_path}")
    print("Generated local NED waypoints:")
    for waypoint in waypoints:
        print(f"  {waypoint}")


def print_coordinate_summary(planner_config):
    gazebo_origin = planner_config.get("gazebo_world_origin_m", [0.0, 0.0, 0.0])
    start_local = cell_to_local_waypoint(
        planner_config["start"],
        planner_config["resolution_m"],
        planner_config["altitude_m"],
    )
    goal_local = cell_to_local_waypoint(
        planner_config["goal"],
        planner_config["resolution_m"],
        planner_config["altitude_m"],
    )
    print("\nCoordinate convention:")
    print(
        "  Gazebo world x = local east + "
        f"map origin x ({gazebo_origin[0]:g} m)"
    )
    print(
        "  Gazebo world y = local north + "
        f"map origin y ({gazebo_origin[1]:g} m)"
    )
    print("  A* grid x = east")
    print("  A* grid y = north")
    print("  MAVSDK local NED: north=grid y, east=grid x, down=-altitude")
    print("  Cell-center conversion enabled.")
    print("\nStart:")
    print(
        f"  grid {list(planner_config['start'])} -> "
        f"local east={start_local['east_m']:.3f}, "
        f"north={start_local['north_m']:.3f}, "
        f"down={start_local['down_m']:.3f}"
    )
    print("\nGoal:")
    print(
        f"  grid {list(planner_config['goal'])} -> "
        f"local east={goal_local['east_m']:.3f}, "
        f"north={goal_local['north_m']:.3f}, "
        f"down={goal_local['down_m']:.3f}"
    )


def reversed_waypoints(waypoints):
    return [waypoint.copy() for waypoint in reversed(waypoints)]


def cells_adjacent_to(cell, width, height):
    x, y = cell
    adjacent = set()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            candidate = (x + dx, y + dy)
            if 0 <= candidate[0] < width and 0 <= candidate[1] < height:
                adjacent.add(candidate)
    return adjacent


def validate_path_clearance(label, path, raw_cells, inflated_cells):
    warnings = []
    raw_entries = [cell for cell in path if cell in raw_cells]
    inflated_entries = [cell for cell in path if cell in inflated_cells]
    if raw_entries:
        warnings.append(f"{label} path enters raw physical footprint cells: {raw_entries}")
    if inflated_entries:
        warnings.append(f"{label} path enters inflated planning obstacle cells: {inflated_entries}")
    return warnings


def validate_planned_routes(grid_path, return_grid_path, planner_config):
    """Return non-fatal warnings about planned route clearance and map quality."""
    warnings = []
    start = planner_config["start"]
    goal = planner_config["goal"]
    width = planner_config["width"]
    height = planner_config["height"]
    raw_cells = planner_config["raw_obstacle_cells"]
    inflated_cells = planner_config["inflated_blocking_cells"]

    if start in raw_cells:
        warnings.append(f"start cell {start} is inside a raw physical footprint")
    if goal in raw_cells:
        warnings.append(f"goal cell {goal} is inside a raw physical footprint")
    if start in inflated_cells:
        warnings.append(f"start cell {start} is inside inflated planning obstacle cells")
    if goal in inflated_cells:
        warnings.append(f"goal cell {goal} is inside inflated planning obstacle cells")
    if cells_adjacent_to(goal, width, height) & inflated_cells:
        candidates = []
        for x in range(max(0, goal[0] - 3), min(width, goal[0] + 4)):
            for y in range(max(0, goal[1] - 3), min(height, goal[1] + 4)):
                cell = (x, y)
                if cell in raw_cells or cell in inflated_cells:
                    continue
                if cells_adjacent_to(cell, width, height) & inflated_cells:
                    continue
                candidates.append(cell)
        warnings.append(
            f"goal cell {goal} is adjacent to inflated planning obstacle cells; "
            f"nearby safer candidate goal cells: {candidates[:6]}"
        )

    warnings.extend(validate_path_clearance("Outbound", grid_path, raw_cells, inflated_cells))
    if return_grid_path:
        warnings.extend(validate_path_clearance("Return", return_grid_path, raw_cells, inflated_cells))
    warnings.extend(map_quality_warnings(grid_path, planner_config))
    return warnings


def is_border_cell(cell, width, height):
    x, y = cell
    return x == 0 or y == 0 or x == width - 1 or y == height - 1


def nearest_inflated_distance(cell, inflated_cells):
    if not inflated_cells:
        return None
    return min(
        max(abs(cell[0] - obstacle[0]), abs(cell[1] - obstacle[1]))
        for obstacle in inflated_cells
    )


def map_quality_warnings(grid_path, planner_config):
    warnings = []
    width = planner_config["width"]
    height = planner_config["height"]
    inflated_cells = planner_config["inflated_blocking_cells"]
    occupancy_ratio = len(inflated_cells) / (width * height)
    if occupancy_ratio > 0.45:
        warnings.append(
            f"inflated planning obstacles occupy {occupancy_ratio:.1%} of the grid; "
            "internal corridors may be over-blocked"
        )

    if grid_path:
        border_cells = [cell for cell in grid_path if is_border_cell(cell, width, height)]
        border_ratio = len(border_cells) / len(grid_path)
        if border_ratio > 0.35:
            warnings.append(
                f"A* route spends {border_ratio:.1%} of its cells on the outer border; "
                "map may still be forcing a boundary-hugging route"
            )

        internal_cells = [
            cell
            for cell in grid_path
            if 2 <= cell[0] <= width - 3 and 2 <= cell[1] <= height - 3
        ]
        internal_ratio = len(internal_cells) / len(grid_path)
        if internal_ratio < 0.25:
            warnings.append(
                f"A* route only spends {internal_ratio:.1%} of its cells in the internal map area; "
                "no meaningful internal corridor was found"
            )

        distances = [
            nearest_inflated_distance(cell, inflated_cells)
            for cell in grid_path
            if cell not in {planner_config["start"], planner_config["goal"]}
        ]
        distances = [distance for distance in distances if distance is not None]
        if distances and min(distances) < 1:
            warnings.append(
                "A* route has a segment with less than 1 cell clearance from inflated cells"
            )
    return warnings


def update_latest(latest, key, value):
    latest[key] = value
    latest["updated_at"][key] = asyncio.get_running_loop().time()


def telemetry_age_s(latest, key):
    updated_at = latest.get("updated_at", {}).get(key)
    if updated_at is None:
        return None
    return asyncio.get_running_loop().time() - updated_at


def ensure_critical_telemetry_fresh(latest, timeout_s):
    if latest.get("connected") is False:
        raise ConnectionError("PX4 connection was lost during flight")
    age_s = telemetry_age_s(latest, "position_velocity")
    if age_s is None:
        raise TimeoutError("Local position telemetry has not been received")
    if age_s > timeout_s:
        raise TimeoutError(
            f"Local position telemetry is stale ({age_s:.1f}s > {timeout_s:.1f}s)"
        )


async def watch_connection_state(drone, latest):
    async for state in drone.core.connection_state():
        update_latest(latest, "connected", bool(state.is_connected))


async def watch_position_velocity_ned(drone, latest):
    """Subscribe to MAVSDK local position/velocity telemetry.

    Side effects:
        Updates `latest["position_velocity"]` until the task is cancelled.
    """
    async for position_velocity in drone.telemetry.position_velocity_ned():
        update_latest(latest, "position_velocity", position_velocity)


async def watch_attitude(drone, latest):
    """Subscribe to MAVSDK attitude telemetry and cache the latest Euler angles."""
    async for attitude in drone.telemetry.attitude_euler():
        update_latest(latest, "attitude", attitude)


async def watch_battery(drone, latest):
    async for battery in drone.telemetry.battery():
        update_latest(latest, "battery", battery)


async def watch_flight_mode(drone, latest):
    async for flight_mode in drone.telemetry.flight_mode():
        update_latest(latest, "flight_mode", flight_mode)


async def watch_armed(drone, latest):
    async for armed in drone.telemetry.armed():
        update_latest(latest, "armed", armed)


async def watch_in_air(drone, latest):
    async for in_air in drone.telemetry.in_air():
        update_latest(latest, "in_air", in_air)


def value_or_blank(message, attribute):
    if message is None:
        return ""
    return getattr(message, attribute, "")


def local_position(latest):
    position_velocity = latest["position_velocity"]
    if position_velocity is None:
        return None
    return getattr(position_velocity, "position", None)


def local_velocity(latest):
    position_velocity = latest["position_velocity"]
    if position_velocity is None:
        return None
    return getattr(position_velocity, "velocity", None)


def target_errors(position, target):
    """Compute local NED tracking error from the current position to a waypoint."""
    if position is None or target["name"] == "":
        return None

    error_north = target["north_m"] - position.north_m
    error_east = target["east_m"] - position.east_m
    error_down = target["down_m"] - position.down_m
    horizontal_error = sqrt(error_north**2 + error_east**2)
    return {
        "north_m": error_north,
        "east_m": error_east,
        "down_m": error_down,
        "horizontal_m": horizontal_error,
    }


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def velocity_command_from_error(error, speed_scale=1.0):
    """Build a bounded PX4 Offboard velocity command from waypoint error.

    The controller is intentionally simple: proportional horizontal/vertical
    velocity with caps. `speed_scale` lets perception slow-down reduce only the
    horizontal speed limit while keeping the waypoint target unchanged.
    """
    north_velocity = POSITION_GAIN * error["north_m"]
    east_velocity = POSITION_GAIN * error["east_m"]
    horizontal_speed = sqrt(north_velocity**2 + east_velocity**2)
    max_horizontal_speed = MAX_HORIZONTAL_SPEED_M_S * speed_scale

    if horizontal_speed > max_horizontal_speed:
        scale = max_horizontal_speed / horizontal_speed
        north_velocity *= scale
        east_velocity *= scale

    down_velocity = clamp(
        POSITION_GAIN * error["down_m"],
        -MAX_VERTICAL_SPEED_M_S,
        MAX_VERTICAL_SPEED_M_S,
    )
    return VelocityNedYaw(north_velocity, east_velocity, down_velocity, 0.0)


def horizontal_distance_to_waypoint(position, waypoint):
    if position is None:
        return None
    north_error = waypoint["north_m"] - position.north_m
    east_error = waypoint["east_m"] - position.east_m
    return sqrt(north_error**2 + east_error**2)


def horizontal_command_speed(command):
    if command is None:
        return None
    north_m_s = getattr(command, "north_m_s", None)
    east_m_s = getattr(command, "east_m_s", None)
    if north_m_s is None or east_m_s is None:
        return None
    return sqrt(north_m_s**2 + east_m_s**2)


def risk_adjusted_speed_scale(base_speed_scale, risk_level, risk_action):
    """Return the speed scale after applying perception risk response.

    Only `slow_down` changes speed. `log_only` and other actions leave the
    command speed unchanged so they can serve as behavior-preserving controls.
    """
    if risk_action != "slow_down":
        return base_speed_scale
    base_speed_m_s = MAX_HORIZONTAL_SPEED_M_S * base_speed_scale
    if risk_level == "danger":
        adjusted_speed_m_s = max(base_speed_m_s * 0.25, MIN_RISK_SPEED_M_S)
        return adjusted_speed_m_s / MAX_HORIZONTAL_SPEED_M_S
    if risk_level == "warning":
        adjusted_speed_m_s = max(base_speed_m_s * 0.5, MIN_RISK_SPEED_M_S)
        return adjusted_speed_m_s / MAX_HORIZONTAL_SPEED_M_S
    return base_speed_scale


def parse_waypoint_timeout(value):
    if str(value).lower() == "auto":
        return "auto"
    try:
        timeout_s = float(value)
    except ValueError as error:
        raise ValueError("--waypoint-timeout must be 'auto' or a positive number") from error
    if timeout_s <= 0:
        raise ValueError("--waypoint-timeout must be 'auto' or a positive number")
    return timeout_s


def waypoint_timeout_info(position, waypoint, speed_scale, perception_config):
    """Estimate a per-waypoint timeout that accounts for configured slow-down."""
    distance_m = horizontal_distance_to_waypoint(position, waypoint)
    if distance_m is None:
        distance_m = 0.0

    expected_speed_m_s = MAX_HORIZONTAL_SPEED_M_S * speed_scale
    if perception_config.get("risk_action", "log_only") == "slow_down":
        expected_speed_m_s = max(expected_speed_m_s * 0.35, MIN_RISK_SPEED_M_S)

    if WAYPOINT_TIMEOUT_MODE == "auto":
        base_timeout_s = distance_m / max(expected_speed_m_s, 0.2)
        timeout_s = max(20.0, base_timeout_s * 3.0 + 10.0)
        if perception_config.get("risk_action", "log_only") == "slow_down":
            timeout_s *= 2.0
    else:
        timeout_s = WAYPOINT_TIMEOUT_MODE

    return {
        "distance_m": distance_m,
        "expected_speed_m_s": expected_speed_m_s,
        "timeout_s": timeout_s,
    }


def print_waypoint_timeout_info(waypoint, timeout_info):
    print(f"Flying to {waypoint['name']}...")
    print(f"  distance: {timeout_info['distance_m']:.2f} m")
    print(f"  expected speed: {timeout_info['expected_speed_m_s']:.2f} m/s")
    print(f"  timeout: {timeout_info['timeout_s']:.1f} s")


def print_waypoint_timeout_debug(waypoint, latest, perception_config, perception_detector, command):
    position = local_position(latest)
    error = target_errors(position, waypoint)
    detection = current_perception_detection(
        perception_config,
        perception_detector,
        position,
        latest["attitude"],
    )
    nearest = detection["nearest_obstacle"] if detection else None
    command_speed = horizontal_command_speed(command)

    print("Waypoint timeout debug:")
    print(f"  waypoint: {waypoint['name']}")
    print(
        "  target N/E/D: "
        f"{waypoint['north_m']:.2f}, {waypoint['east_m']:.2f}, {waypoint['down_m']:.2f}"
    )
    if position is None:
        print("  latest local N/E/D: unavailable")
        print("  current horizontal error: unavailable")
    else:
        print(
            "  latest local N/E/D: "
            f"{position.north_m:.2f}, {position.east_m:.2f}, {position.down_m:.2f}"
        )
        print(f"  current horizontal error: {error['horizontal_m']:.2f} m")
    print(f"  latest perception_risk_level: {detection['risk_level'] if detection else 'clear'}")
    if nearest is None:
        print("  latest nearest obstacle: unavailable")
    else:
        print(
            "  latest nearest obstacle: "
            f"{nearest['obstacle_name']} at {nearest['distance_m']:.2f} m"
        )
    if command_speed is None:
        print("  commanded speed: unavailable")
    else:
        print(f"  commanded speed: {command_speed:.2f} m/s")


def set_phase(phase_state, phase, route_direction="none"):
    phase_state["phase"] = phase
    phase_state["route_direction"] = route_direction


def current_perception_detection(
    perception_config,
    detector,
    position,
    attitude,
    timestamp_utc=None,
    elapsed_s=None,
    replan_config=None,
):
    """Return the structured perception state for the current telemetry sample.

    Args:
        perception_config: CLI-derived perception settings.
        detector: The simulated map-based detector, or `None` when disabled.
        position: Current MAVSDK local NED position.
        attitude: Current MAVSDK attitude message.
        timestamp_utc: Optional timestamp for telemetry rows.
        elapsed_s: Optional elapsed time for telemetry rows.
        replan_config: Optional local replan settings used to label suggested
            actions such as `replan_candidate`.

    Returns:
        A perception state dictionary. Legacy detector keys are preserved so
        existing logging and replan code can consume the same object.
    """
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


def local_position_to_grid_cell(position, resolution_m):
    if position is None or resolution_m <= 0:
        return None
    return (
        int(floor(position.east_m / resolution_m)),
        int(floor(position.north_m / resolution_m)),
    )


def empty_replan_state():
    return {
        "replan_mode": "log_only",
        "replan_triggered": False,
        "replan_success": "",
        "replan_start_grid_x": "",
        "replan_start_grid_y": "",
        "replan_goal_grid_x": "",
        "replan_goal_grid_y": "",
        "replan_path_length": "",
        "dynamic_blocked_cell_count": "",
        "replan_count": 0,
        "replan_route_replaced": False,
        "active_replan_count": 0,
        "active_replan_path_length": "",
        "last_attempt_time": None,
    }


def reset_replan_event_fields(replan_state):
    replan_state["replan_triggered"] = False
    replan_state["replan_success"] = ""
    replan_state["replan_start_grid_x"] = ""
    replan_state["replan_start_grid_y"] = ""
    replan_state["replan_goal_grid_x"] = ""
    replan_state["replan_goal_grid_y"] = ""
    replan_state["replan_path_length"] = ""
    replan_state["dynamic_blocked_cell_count"] = ""
    replan_state["replan_route_replaced"] = False
    replan_state["active_replan_path_length"] = ""


def dynamic_cells_from_detection(detection):
    """Extract grid cells from detected obstacles for local replan blocking."""
    if not detection:
        return set()
    cells = set()
    for obstacle in detection.get("detected_obstacles", []):
        grid_x = obstacle.get("grid_x")
        grid_y = obstacle.get("grid_y")
        if grid_x is None or grid_y is None:
            continue
        cells.add((int(grid_x), int(grid_y)))
    return cells


def should_attempt_local_replan(replan_config, replan_state, risk_level, now_s):
    """Decide whether the current risk sample should trigger a local A* replan."""
    if not replan_config.get("enabled"):
        return False
    if replan_state.get("replan_count", 0) >= replan_config["max_replans"]:
        return False
    if not risk_reaches_threshold(risk_level, replan_config["risk_level"]):
        return False
    last_attempt_time = replan_state.get("last_attempt_time")
    if (
        last_attempt_time is not None
        and now_s - last_attempt_time < replan_config["cooldown_s"]
    ):
        return False
    return True


def attempt_local_replan(replan_config, replan_state, position, detection, now_s):
    """Attempt a local A* replan from current position to the original goal.

    Detected obstacle cells are inflated and merged with the static obstacle
    map. The function updates `replan_state` for telemetry logging and returns
    the raw replanned grid path on success, or `None` on failure.
    """
    replan_state["last_attempt_time"] = now_s
    replan_state["replan_count"] = replan_state.get("replan_count", 0) + 1
    reset_replan_event_fields(replan_state)
    replan_state["replan_triggered"] = True

    start_cell = local_position_to_grid_cell(position, replan_config["resolution_m"])
    goal_cell = replan_config["goal_cell"]
    if start_cell is not None:
        replan_state["replan_start_grid_x"] = start_cell[0]
        replan_state["replan_start_grid_y"] = start_cell[1]
    replan_state["replan_goal_grid_x"] = goal_cell[0]
    replan_state["replan_goal_grid_y"] = goal_cell[1]

    dynamic_cells = dynamic_cells_from_detection(detection)
    inflated_dynamic_cells = inflate_cells(
        dynamic_cells,
        replan_config["width"],
        replan_config["height"],
        replan_config["dynamic_obstacle_inflation"],
    )
    replan_state["dynamic_blocked_cell_count"] = len(inflated_dynamic_cells)

    if start_cell is None:
        replan_state["replan_success"] = "false"
        print("Local replan attempt failed: current local position is unavailable.")
        return None

    planning_obstacles = set(replan_config["static_obstacles"]) | inflated_dynamic_cells
    # Keep the temporary replan start/goal free so A* can evaluate whether a path
    # exists from the current cell even when those cells overlap inflated obstacles.
    planning_obstacles -= {start_cell, goal_cell}

    try:
        replanned_path = astar(
            start=start_cell,
            goal=goal_cell,
            obstacles=planning_obstacles,
            width=replan_config["width"],
            height=replan_config["height"],
            allow_diagonal=replan_config["allow_diagonal"],
        )
    except ValueError as error:
        replan_state["replan_success"] = "false"
        print(
            "Local replan attempt failed: "
            f"start={start_cell}, goal={goal_cell}, dynamic_cells={len(inflated_dynamic_cells)}, "
            f"error={error}"
        )
        return None

    replan_state["replan_success"] = "true"
    replan_state["replan_path_length"] = len(replanned_path)
    print(
        "Local replan attempt succeeded: "
        f"start={start_cell}, goal={goal_cell}, dynamic_cells={len(inflated_dynamic_cells)}, "
        f"path_length={len(replanned_path)}"
    )
    return replanned_path


def replanned_path_to_waypoints(replanned_path, replan_config):
    """Convert a successful local replan grid path into replacement waypoints."""
    simplified_path = simplify_grid_path(replanned_path)
    waypoints = grid_path_to_local_waypoints(
        simplified_path,
        replan_config["resolution_m"],
        replan_config["altitude_m"],
    )
    for index, waypoint in enumerate(waypoints, start=1):
        waypoint["name"] = f"RWP{index:02d}"
    return waypoints


def first_useful_replan_waypoint_index(position, waypoints):
    """Skip replacement waypoints that are already inside acceptance tolerance."""
    if position is None:
        return 0
    for index, waypoint in enumerate(waypoints):
        error = target_errors(position, waypoint)
        if not error:
            return index
        if (
            error["horizontal_m"] >= REACHED_HORIZONTAL_ERROR_M
            or abs(error["down_m"]) >= REACHED_VERTICAL_ERROR_M
        ):
            return index
    return max(len(waypoints) - 1, 0)


def build_active_replan_route(replanned_path, replan_config, replan_state, position):
    """Build and record the outbound waypoint replacement for active replan mode.

    Side effects:
        Marks `replan_route_replaced`, increments active replan counters, and
        records path length for telemetry and analysis.
    """
    replanned_waypoints = replanned_path_to_waypoints(replanned_path, replan_config)
    if not replanned_waypoints:
        return None

    first_index = first_useful_replan_waypoint_index(position, replanned_waypoints)
    replacement_waypoints = replanned_waypoints[first_index:]
    if not replacement_waypoints:
        replacement_waypoints = [replanned_waypoints[-1]]

    replan_state["replan_route_replaced"] = True
    replan_state["active_replan_count"] = replan_state.get("active_replan_count", 0) + 1
    replan_state["active_replan_path_length"] = len(replanned_path)
    print(
        "Active local replan replaced outbound route: "
        f"replacement_waypoints={len(replacement_waypoints)}, "
        f"grid_path_length={len(replanned_path)}, skipped_waypoints={first_index}"
    )
    return replacement_waypoints


async def log_telemetry(
    drone,
    stop_event,
    log_path,
    latest,
    phase_state,
    target_state,
    planner_info,
    perception_config,
    replan_config,
    replan_state,
    perception_detector=None,
):
    """Write live PX4 telemetry, perception state, and replan state to CSV.

    The coroutine owns MAVSDK telemetry subscription tasks and cancels them on
    exit. It also clears one-shot replan event fields after a row has captured
    them so trigger/replacement events are visible in logs without persisting
    forever.
    """
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    start_time = datetime.now(timezone.utc)

    watcher_tasks = [
        asyncio.create_task(watch_connection_state(drone, latest), name="connection-state"),
        asyncio.create_task(
            watch_position_velocity_ned(drone, latest), name="position-velocity"
        ),
        asyncio.create_task(watch_attitude(drone, latest), name="attitude"),
        asyncio.create_task(watch_battery(drone, latest), name="battery"),
        asyncio.create_task(watch_flight_mode(drone, latest), name="flight-mode"),
        asyncio.create_task(watch_armed(drone, latest), name="armed"),
        asyncio.create_task(watch_in_air(drone, latest), name="in-air"),
    ]

    with log_path.open("w", newline="") as log_file:
        writer = csv.writer(log_file)
        writer.writerow(TELEMETRY_CSV_HEADER)

        try:
            while not stop_event.is_set():
                for task in watcher_tasks:
                    if not task.done():
                        continue
                    if task.cancelled():
                        raise RuntimeError(
                            f"Telemetry watcher {task.get_name()} was cancelled unexpectedly"
                        )
                    error = task.exception()
                    if error is not None:
                        raise RuntimeError(
                            f"Telemetry watcher {task.get_name()} failed"
                        ) from error
                    raise RuntimeError(
                        f"Telemetry watcher {task.get_name()} stopped unexpectedly"
                    )
                now = datetime.now(timezone.utc)
                position = local_position(latest)
                velocity = local_velocity(latest)
                attitude = latest["attitude"]
                battery = latest["battery"]
                target = target_state.copy()
                error = target_errors(position, target)
                detection = current_perception_detection(
                    perception_config,
                    perception_detector,
                    position,
                    attitude,
                    timestamp_utc=now.isoformat(),
                    elapsed_s=round((now - start_time).total_seconds(), 3),
                    replan_config=replan_config,
                )
                perception_values = perception_csv_values(perception_config, detection)
                replan_values = replan_csv_values(replan_state)

                writer.writerow(
                    build_telemetry_log_row(
                        now,
                        start_time,
                        phase_state,
                        target,
                        position,
                        velocity,
                        attitude,
                        battery,
                        latest["flight_mode"],
                        latest["armed"],
                        planner_info,
                        error,
                        perception_values,
                        replan_values,
                    )
                )
                if replan_state.get("replan_triggered") or replan_state.get("replan_route_replaced"):
                    reset_replan_event_fields(replan_state)
                log_file.flush()
                await asyncio.sleep(0.2)
        finally:
            pending_watchers = await cancel_tasks(
                watcher_tasks,
                LOGGER_SHUTDOWN_TIMEOUT_S,
            )
            if pending_watchers:
                names = ", ".join(task.get_name() for task in pending_watchers)
                raise TimeoutError(
                    f"Telemetry watchers did not stop before timeout: {names}"
                )


async def wait_for_connection(drone, timeout_s=CONNECTION_TIMEOUT_S):
    print("Waiting for drone connection...")
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout_s
    connection_stream = drone.core.connection_state().__aiter__()
    while True:
        remaining_s = deadline - loop.time()
        if remaining_s <= 0:
            raise TimeoutError(f"Timed out waiting {timeout_s:g}s for PX4 connection")
        try:
            state = await asyncio.wait_for(connection_stream.__anext__(), timeout=remaining_s)
        except asyncio.TimeoutError as error:
            raise TimeoutError(
                f"Timed out waiting {timeout_s:g}s for PX4 connection"
            ) from error
        except StopAsyncIteration as error:
            raise ConnectionError("PX4 connection stream ended before connecting") from error
        if state.is_connected:
            print("Connected to drone.")
            return


def health_status_text(health):
    if health is None:
        return (
            "local_position_ok=unknown, "
            "global_position_ok=unknown, "
            "home_position_ok=unknown"
        )
    return (
        f"local_position_ok={health.is_local_position_ok}, "
        f"global_position_ok={health.is_global_position_ok}, "
        f"home_position_ok={health.is_home_position_ok}"
    )


async def wait_for_position_ready(drone, timeout_s=60):
    """Wait until PX4 reports local-position readiness for NED waypoint flight."""
    print("Waiting for PX4 position readiness...")
    print(
        "This A* experiment flies local NED waypoints, so local position is the main requirement."
    )

    # Global position is GPS-like position readiness.
    # Home position is the takeoff/home reference PX4 uses for many global missions.
    # Local position is enough for local NED Offboard waypoint experiments in PX4 SITL.
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout_s
    next_status_print = 0.0
    latest_health = None
    health_stream = drone.telemetry.health().__aiter__()

    while True:
        remaining_s = deadline - loop.time()
        if remaining_s <= 0:
            raise TimeoutError(
                "Timed out waiting for local position readiness. "
                f"Latest health: {health_status_text(latest_health)}"
            )

        try:
            latest_health = await asyncio.wait_for(
                health_stream.__anext__(),
                timeout=remaining_s,
            )
        except asyncio.TimeoutError as error:
            raise TimeoutError(
                "Timed out waiting for local position readiness. "
                f"Latest health: {health_status_text(latest_health)}"
            ) from error
        except StopAsyncIteration as error:
            raise ConnectionError(
                "PX4 health stream ended before local position became ready"
            ) from error

        now = loop.time()
        if now >= next_status_print:
            print(f"Position health: {health_status_text(latest_health)}")
            next_status_print = now + 3

        if latest_health.is_local_position_ok:
            if latest_health.is_global_position_ok and latest_health.is_home_position_ok:
                print("Full global/home position is ready.")
            else:
                print(
                    "Local position is OK. Continuing because this experiment uses local NED waypoints."
                )
            return


async def wait_for_local_position(latest, timeout_s=TELEMETRY_TIMEOUT_S):
    print("Waiting for local NED position telemetry...")
    deadline = asyncio.get_running_loop().time() + timeout_s
    while local_position(latest) is None:
        if asyncio.get_running_loop().time() >= deadline:
            raise TimeoutError(
                f"Timed out waiting {timeout_s:g}s for local NED position telemetry"
            )
        await asyncio.sleep(0.2)
    print("Local NED position telemetry is available.")


async def wait_until_landed(latest, timeout_s=45):
    print("Waiting for landing to finish...")
    deadline = asyncio.get_running_loop().time() + timeout_s
    while asyncio.get_running_loop().time() < deadline:
        if latest["in_air"] is False:
            print("Drone has landed.")
            return True
        await asyncio.sleep(1)
    raise TimeoutError(
        f"Landing was not confirmed within {timeout_s:g}s; PX4 may still be landing"
    )


async def fly_to_waypoint(
    drone,
    latest,
    phase_state,
    target_state,
    waypoint,
    phase_name,
    route_direction,
    speed_scale,
    perception_config,
    perception_detector,
    replan_config,
    replan_state,
):
    """Track one local NED waypoint with perception and optional local replan.

    Returns:
        `None` when the waypoint is reached normally, or a replacement waypoint
        list when active local replan replaces the remaining outbound route.

    Side effects:
        Sends PX4 Offboard velocity commands and may raise
        `DangerObstacleDetected` for the `stop_and_land` risk action.
    """
    set_phase(phase_state, phase_name, route_direction)
    target_state.update(waypoint)

    timeout_info = waypoint_timeout_info(
        local_position(latest),
        waypoint,
        speed_scale,
        perception_config,
    )
    print_waypoint_timeout_info(waypoint, timeout_info)

    last_command = None
    deadline = asyncio.get_running_loop().time() + timeout_info["timeout_s"]
    while asyncio.get_running_loop().time() < deadline:
        ensure_critical_telemetry_fresh(latest, TELEMETRY_TIMEOUT_S)
        position = local_position(latest)
        if position is None:
            last_command = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            await drone.offboard.set_velocity_ned(last_command)
            await asyncio.sleep(0.2)
            continue

        error = target_errors(position, waypoint)
        detection = current_perception_detection(
            perception_config,
            perception_detector,
            position,
            latest["attitude"],
            replan_config=replan_config,
        )
        risk_level = detection["risk_level"] if detection else "clear"
        now_s = asyncio.get_running_loop().time()
        # Local replan is event-triggered by risk level, rate-limited by
        # cooldown, and capped by max_replans so one obstacle cluster cannot
        # spam the planner.
        if should_attempt_local_replan(replan_config, replan_state, risk_level, now_s):
            replanned_path = attempt_local_replan(
                replan_config,
                replan_state,
                position,
                detection,
                now_s,
            )
            if (
                route_direction == "outbound"
                and replan_config.get("mode") == "active"
                and replanned_path
            ):
                # Active mode is deliberately limited to outbound flight. The
                # return route remains the deterministic reversed A* path.
                replacement_waypoints = build_active_replan_route(
                    replanned_path,
                    replan_config,
                    replan_state,
                    position,
                )
                if replacement_waypoints:
                    last_command = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                    await drone.offboard.set_velocity_ned(last_command)
                    return replacement_waypoints

        risk_action = perception_config.get("risk_action", "log_only")
        if risk_action == "stop_and_land" and risk_level == "danger":
            last_command = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            await drone.offboard.set_velocity_ned(last_command)
            raise DangerObstacleDetected

        if (
            error["horizontal_m"] < REACHED_HORIZONTAL_ERROR_M
            and abs(error["down_m"]) < REACHED_VERTICAL_ERROR_M
        ):
            # A waypoint is accepted only when horizontal and vertical errors
            # are both inside tolerance; this prevents early phase switching
            # while still allowing small PX4/SITL tracking noise.
            last_command = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            await drone.offboard.set_velocity_ned(last_command)
            print(f"Reached {waypoint['name']}.")
            await asyncio.sleep(TURN_SETTLE_S)
            return None

        adjusted_speed_scale = risk_adjusted_speed_scale(
            speed_scale,
            risk_level,
            risk_action,
        )
        last_command = velocity_command_from_error(error, adjusted_speed_scale)
        await drone.offboard.set_velocity_ned(last_command)
        await asyncio.sleep(0.2)

    print_waypoint_timeout_debug(
        waypoint,
        latest,
        perception_config,
        perception_detector,
        last_command,
    )
    raise TimeoutError(f"Timed out before reaching {waypoint['name']}")


async def fly_waypoint_route(
    drone,
    latest,
    phase_state,
    target_state,
    waypoints,
    phase_name,
    route_direction,
    perception_config,
    perception_detector,
    replan_config,
    replan_state,
    speed_scale=1.0,
):
    """Fly a sequence of waypoints, allowing active replan to swap the sequence."""
    active_waypoints = list(waypoints)
    waypoint_index = 0
    while waypoint_index < len(active_waypoints):
        waypoint = active_waypoints[waypoint_index]
        replacement_waypoints = await fly_to_waypoint(
            drone,
            latest,
            phase_state,
            target_state,
            waypoint,
            phase_name,
            route_direction,
            speed_scale,
            perception_config,
            perception_detector,
            replan_config,
            replan_state,
        )
        if (
            route_direction == "outbound"
            and replan_config.get("mode") == "active"
            and replacement_waypoints
        ):
            # Restart indexing against the replacement list so the next loop
            # target is the first useful replanned waypoint.
            active_waypoints = list(replacement_waypoints)
            waypoint_index = 0
            print(
                "Continuing outbound flight on active replanned route "
                f"with {len(active_waypoints)} waypoint(s)."
            )
            continue
        waypoint_index += 1


async def hover_at_waypoint(
    drone,
    phase_state,
    target_state,
    waypoint,
    phase_name,
    hover_s,
):
    set_phase(phase_state, phase_name)
    target_state.update(waypoint)
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(hover_s)


async def fly_astar_waypoints(
    drone,
    latest,
    phase_state,
    target_state,
    waypoints,
    perception_config,
    perception_detector,
    replan_config,
    replan_state,
    return_home=False,
):
    """Execute the complete takeoff, outbound route, optional return, and landing."""

    target_takeoff_altitude_m = abs(float(waypoints[0]["down_m"]))

    print(f"Setting PX4 takeoff altitude to {target_takeoff_altitude_m:.2f} m...")
    await drone.action.set_takeoff_altitude(target_takeoff_altitude_m)

    set_phase(phase_state, "takeoff")
    print("Arming...")
    await drone.action.arm()

    set_phase(phase_state, "takeoff")
    print("Taking off...")
    await drone.action.takeoff()

    set_phase(phase_state, "takeoff")
    print("Waiting 8 seconds for takeoff stabilization...")
    await asyncio.sleep(8)
    await wait_for_local_position(latest, TELEMETRY_TIMEOUT_S)

    print("Sending initial zero velocity setpoint before Offboard start...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    set_phase(phase_state, "takeoff")
    print("Starting Offboard mode...")
    await drone.offboard.start()
    print("Offboard mode started.")

    print("Flying outbound A* path to goal...")
    await fly_waypoint_route(
        drone,
        latest,
        phase_state,
        target_state,
        waypoints,
        "outbound_to_goal",
        "outbound",
        perception_config,
        perception_detector,
        replan_config,
        replan_state,
        1.0,
    )
    print("Reached goal.")

    print("Hovering briefly at the goal...")
    await hover_at_waypoint(
        drone,
        phase_state,
        target_state,
        waypoints[-1],
        "goal_hover",
        3,
    )

    if return_home:
        return_waypoints = reversed_waypoints(waypoints)
        print("Return-home enabled. Flying reversed path back to start...")
        print(f"Return route speed scale: {RETURN_SPEED_SCALE:.2f}")
        await fly_waypoint_route(
            drone,
            latest,
            phase_state,
            target_state,
            return_waypoints,
            "return_to_start",
            "return",
            perception_config,
            perception_detector,
            replan_config,
            replan_state,
            RETURN_SPEED_SCALE,
        )
        print("Reached start area.")

        print("Hovering briefly at the start area...")
        await hover_at_waypoint(
            drone,
            phase_state,
            target_state,
            return_waypoints[-1],
            "start_hover",
            2,
        )
    else:
        print("Return-home disabled. Landing at goal.")

    set_phase(phase_state, "landing")
    print("Stopping Offboard mode...")
    await drone.offboard.stop()
    print("Offboard mode stopped.")

    set_phase(phase_state, "landing")
    print("Landing...")
    await drone.action.land()
    await wait_until_landed(latest, LANDING_TIMEOUT_S)
    set_phase(phase_state, "landed")


def build_perception_detector(args, planner_config):
    """Create the simulated obstacle detector when perception is enabled."""
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


async def attempt_safe_landing(drone, latest, phase_state, phase_name):
    """Best-effort failsafe landing that reports whether PX4 confirmed touchdown."""
    set_phase(phase_state, phase_name)
    print("Trying to stop Offboard mode and land safely...")
    with contextlib.suppress(Exception):
        await drone.offboard.stop()
    try:
        await drone.action.land()
        await wait_until_landed(latest, LANDING_TIMEOUT_S)
    except Exception as error:
        print(f"Landing was not confirmed: {error}")
        return False
    set_phase(phase_state, "landed")
    return True


async def run_flight(
    system_address,
    waypoints,
    planner_info,
    perception_config,
    replan_config,
    perception_detector=None,
    return_home=False,
):
    """Connect to PX4, start telemetry logging, fly the route, and land safely.

    Side effects:
        Opens a MAVSDK connection, writes a timestamped CSV log under
        `data/logs/`, starts/cancels telemetry tasks, and attempts landing after
        expected or unexpected flight termination.
    """
    drone = System()
    log_path = make_log_path()
    latest = {
        "connected": None,
        "position_velocity": None,
        "attitude": None,
        "battery": None,
        "flight_mode": None,
        "armed": None,
        "in_air": None,
        "updated_at": {},
    }
    replan_state = empty_replan_state()
    replan_state["replan_mode"] = replan_config.get("mode", "log_only")
    phase_state = {"phase": "connecting", "route_direction": "none"}
    target_state = {"name": "", "north_m": 0.0, "east_m": 0.0, "down_m": 0.0}
    stop_logging = asyncio.Event()
    telemetry_task = None
    mission_task = None
    pending_error = None
    landing_confirmed = None
    status_path = write_run_status(log_path, "starting", "connecting")
    print(f"Run status: {status_path}")

    try:
        print(f"Connecting to PX4 SITL with MAVSDK at {system_address}...")
        try:
            await asyncio.wait_for(
                drone.connect(system_address=system_address),
                timeout=CONNECTION_TIMEOUT_S,
            )
        except asyncio.TimeoutError as error:
            raise TimeoutError(
                f"Timed out waiting {CONNECTION_TIMEOUT_S:g}s for MAVSDK connection startup"
            ) from error
        await wait_for_connection(drone, CONNECTION_TIMEOUT_S)
        await wait_for_position_ready(drone, POSITION_READY_TIMEOUT_S)

        print(f"Starting telemetry log: {log_path}")
        telemetry_task = asyncio.create_task(
            log_telemetry(
                drone,
                stop_logging,
                log_path,
                latest,
                phase_state,
                target_state,
                planner_info,
                perception_config,
                replan_config,
                replan_state,
                perception_detector,
            ),
            name="telemetry-logger",
        )
        write_run_status(log_path, "running", phase_state["phase"])
        mission_task = asyncio.create_task(
            fly_astar_waypoints(
                drone,
                latest,
                phase_state,
                target_state,
                waypoints,
                perception_config,
                perception_detector,
                replan_config,
                replan_state,
                return_home=return_home,
            ),
            name="flight-mission",
        )
        done, _ = await asyncio.wait(
            {mission_task, telemetry_task},
            return_when=asyncio.FIRST_COMPLETED,
        )
        if telemetry_task in done:
            if not mission_task.done():
                mission_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await mission_task
            logger_error = telemetry_task.exception()
            if logger_error is not None:
                raise RuntimeError("Telemetry logger failed during flight") from logger_error
            raise RuntimeError("Telemetry logger stopped unexpectedly during flight")
        await mission_task
        landing_confirmed = phase_state["phase"] == "landed"
        if not landing_confirmed:
            raise RuntimeError("Mission ended without confirmed landing")
        write_run_status(
            log_path,
            "completed",
            phase_state["phase"],
            landing_confirmed=True,
        )
    except Exception as error:
        print(f"Flight error: {error}")
        pending_error = error
        phase_name = (
            "landing_after_danger"
            if isinstance(error, DangerObstacleDetected)
            else "landing_after_error"
        )
        if phase_state["phase"] == "landed":
            landing_confirmed = True
        elif telemetry_task is not None:
            landing_confirmed = await attempt_safe_landing(
                drone, latest, phase_state, phase_name
            )
        write_run_status(
            log_path,
            "failed",
            phase_state["phase"],
            message=f"{type(error).__name__}: {error}",
            landing_confirmed=landing_confirmed,
        )
    finally:
        if telemetry_task is not None:
            print("Stopping telemetry logging...")
            stop_logging.set()
            try:
                await asyncio.wait_for(
                    asyncio.shield(telemetry_task),
                    timeout=LOGGER_SHUTDOWN_TIMEOUT_S,
                )
            except Exception as error:
                if not telemetry_task.done():
                    await cancel_tasks(
                        [telemetry_task],
                        LOGGER_SHUTDOWN_TIMEOUT_S,
                    )
                if pending_error is None:
                    pending_error = RuntimeError("Telemetry logger did not shut down cleanly")
                    pending_error.__cause__ = error
                    write_run_status(
                        log_path,
                        "failed",
                        phase_state["phase"],
                        message=f"{type(error).__name__}: {error}",
                        landing_confirmed=landing_confirmed,
                    )
            if log_path.exists():
                print(f"Telemetry log saved to {log_path}")

    if pending_error is not None:
        raise pending_error

    print("Done.")


def main(argv=None):
    """CLI entry point for dry-run preview or live PX4 SITL flight execution."""
    args = parse_args(argv)
    global MAX_HORIZONTAL_SPEED_M_S, REACHED_HORIZONTAL_ERROR_M
    global RETURN_SPEED_SCALE, TURN_SETTLE_S
    global WAYPOINT_TIMEOUT_MODE, MIN_RISK_SPEED_M_S
    global CONNECTION_TIMEOUT_S, POSITION_READY_TIMEOUT_S
    global TELEMETRY_TIMEOUT_S, LANDING_TIMEOUT_S, LOGGER_SHUTDOWN_TIMEOUT_S
    validate_runtime_args(args)

    MAX_HORIZONTAL_SPEED_M_S = args.max_speed
    REACHED_HORIZONTAL_ERROR_M = args.waypoint_acceptance
    RETURN_SPEED_SCALE = args.return_speed_scale
    TURN_SETTLE_S = args.turn_settle
    WAYPOINT_TIMEOUT_MODE = parse_waypoint_timeout(args.waypoint_timeout)
    MIN_RISK_SPEED_M_S = args.min_risk_speed
    CONNECTION_TIMEOUT_S = args.connection_timeout
    POSITION_READY_TIMEOUT_S = args.position_ready_timeout
    TELEMETRY_TIMEOUT_S = args.telemetry_timeout
    LANDING_TIMEOUT_S = args.landing_timeout
    LOGGER_SHUTDOWN_TIMEOUT_S = args.logger_shutdown_timeout

    planner_config = load_planner_config(args)
    validate_planner_safety(planner_config)
    perception_config = build_perception_config(args)
    perception_detector = build_perception_detector(args, planner_config)
    replan_config = build_replan_config(args, planner_config)
    grid_path, simplified_path, waypoints = plan_path(planner_config, args.allow_diagonal)
    return_grid_path = list(reversed(grid_path)) if args.return_home else []
    route_warnings = validate_planned_routes(grid_path, return_grid_path, planner_config)
    if route_warnings:
        planner_config["validation_warnings"] = [
            *planner_config.get("validation_warnings", []),
            *route_warnings,
        ]
        print("Route clearance warnings:")
        for warning in route_warnings:
            print(f"  WARNING: {warning}")
    planner_info = {
        "grid_start": str(planner_config["start"]),
        "grid_goal": str(planner_config["goal"]),
        "grid_width": planner_config["width"],
        "grid_height": planner_config["height"],
        "planner_name": PLANNER_NAME,
        "map_name": planner_config["map_name"],
        "resolution_m": planner_config["resolution_m"],
        "altitude_m": planner_config["altitude_m"],
        "return_home_enabled": args.return_home,
    }

    if args.compact_output:
        print(
            f"Route summary: {len(grid_path)} grid cells, "
            f"{len(waypoints)} flight waypoints"
        )
        print(
            "Perception: "
            + (
                f"enabled ({perception_config['risk_action']})"
                if perception_config["enabled"]
                else "disabled"
            )
        )
        print(
            "Local replanning: "
            + (
                f"enabled ({replan_config['mode']})"
                if replan_config["enabled"]
                else "disabled"
            )
        )
    else:
        print_coordinate_summary(planner_config)
        print_plan(grid_path, simplified_path, waypoints)
        print_perception_summary(perception_config)
        print_replan_summary(replan_config)

    if args.dry_run:
        print("Dry run requested: not connecting to PX4 and not flying.")
        if args.return_home:
            print("Return-home preview requested.")
        save_preview(
            grid_path,
            simplified_path,
            waypoints,
            args,
            planner_config,
            PREVIEW_DIR,
            PLANNER_NAME,
            display_path,
            reversed_waypoints,
        )
        return

    run_with_bounded_shutdown(
        run_flight(
            args.system_address,
            waypoints,
            planner_info,
            perception_config,
            replan_config,
            perception_detector,
            return_home=args.return_home,
        ),
        LOGGER_SHUTDOWN_TIMEOUT_S,
    )


if __name__ == "__main__":
    main()
