import argparse
import json
import math
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

import pandas as pd


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.planner.obstacle_config import build_obstacle_map, get_resolution_altitude, load_obstacle_config
from src.logging.collision_checks import obstacle_collision_report
from src.logging.active_replan_validation import validate_active_replan_rows
from src.logging.log_io import (
    display_path,
    prepare_dataframe,
    resolve_log_path,
    resolve_project_path,
    run_id_from_log,
)
from src.logging.metrics import (
    DEFAULT_DANGER_DISTANCE_M,
    DEFAULT_WARNING_DISTANCE_M,
    RISK_LEVEL_TO_VALUE,
    bool_from_value,
    detected_obstacle_mask,
    duration_s,
    first_value,
    first_active_value,
    frequent_obstacle_names,
    has_perception_columns,
    perception_enabled_mask,
    ratio,
    risk_level_series,
    safe_last_valid,
    safe_max,
    safe_median,
    time_in_risk_levels,
)
from src.logging.output_registry import ensure_output_tree, get_run_output_dir


OUTPUT_ROOT = PROJECT_ROOT / "outputs"
MAX_ASTAR_OUTPUT_FOLDERS = 10
ASTAR_OUTPUT_PATTERN = re.compile(r"^as_\d{8}_\d{6}$")
WAYPOINT_REACHED_THRESHOLD_M = 0.4
PLOT_SPLIT_DISTANCE_M = 3.0
# Keep matplotlib cache files inside this project instead of relying on a
# writable home-directory cache.
MPLCONFIG_DIR = OUTPUT_ROOT / ".matplotlib_cache"
MPLCONFIG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIG_DIR))

from src.logging.plotting import (
    save_collision_zoom_plot,
    save_detection_count_plot,
    save_error_plot,
    save_line_plot,
    save_perception_risk_timeline,
    save_perception_timeline,
    save_target_timeline,
    save_trajectory_plot,
    target_sequence,
)
from src.logging.report_writer import (
    save_collision_points_csv,
    write_manifest,
    write_run_metadata,
    write_summary,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Analyze an A* path flight log.")
    parser.add_argument(
        "--log",
        type=Path,
        help="Path to an astar_*.csv log. If omitted, the newest log is used.",
    )
    parser.add_argument(
        "--obstacle-config",
        type=Path,
        help="Optional JSON obstacle config, for example config/substation_obstacles.json.",
    )
    parser.add_argument(
        "--debug-plots",
        action="store_true",
        help=(
            "Also generate altitude, velocity, yaw, waypoint target, nearest-obstacle, "
            "and detection-count debug plots. Default analysis only writes core plots."
        ),
    )
    return parser.parse_args()


def infer_return_home_enabled(df):
    configured = first_value(df, "return_home_enabled")
    if configured is not None:
        return bool_from_value(configured)
    if "route_direction" in df.columns:
        return bool((df["route_direction"] == "return").any())
    return None


def infer_local_replan_enabled(df):
    if "replan_count" in df.columns and safe_max(df, "replan_count"):
        return True
    if "replan_triggered" in df.columns and df["replan_triggered"].map(bool_from_value).fillna(False).any():
        return True
    if "replan_route_replaced" in df.columns and df["replan_route_replaced"].map(bool_from_value).fillna(False).any():
        return True
    if "active_replan_count" in df.columns and safe_max(df, "active_replan_count"):
        return True
    return False


def infer_experiment_stage(df):
    perception_enabled = bool(perception_enabled_mask(df).any()) if "perception_enabled" in df.columns else False
    local_replan_enabled = infer_local_replan_enabled(df)
    replan_mode = str(first_value(df, "replan_mode") or "log_only").strip().lower()
    if local_replan_enabled and replan_mode == "active":
        return "active_replan"
    if local_replan_enabled and replan_mode == "log_only":
        return "replan_log_only"
    if perception_enabled:
        return "perception_response"
    return "static_astar"


def infer_experiment_type(df, stage):
    risk_action = first_active_value(df, "risk_action") or first_value(df, "risk_action")
    if stage == "static_astar":
        return "astar_only_perception_disabled"
    if stage == "active_replan":
        return "active_local_replan"
    if stage == "replan_log_only":
        return "local_replan_log_only"
    if risk_action == "slow_down":
        return "perception_slow_down"
    if risk_action == "log_only":
        return "perception_log_only"
    return "perception_response"


def load_analysis_obstacles(args, df, warnings):
    config_path = resolve_project_path(args.obstacle_config)
    if config_path is None:
        inferred_path = PROJECT_ROOT / "config" / "substation_obstacles.json"
        map_name = first_value(df, "map_name")
        if inferred_path.exists() and map_name:
            try:
                inferred_config = load_obstacle_config(inferred_path)
                if inferred_config.get("map_name") == map_name:
                    config_path = inferred_path
                    print(f"Inferred obstacle config: {display_path(config_path)}")
            except Exception as error:
                warnings.append(f"Could not infer obstacle config: {error}")

    if config_path is None:
        warnings.append("No obstacle config provided; obstacle avoidance could not be validated in analysis.")
        return None, None, set(), None, None

    config = load_obstacle_config(config_path)
    log_altitude_m = first_value(df, "altitude_m")
    _, config_altitude_m = get_resolution_altitude(config)
    altitude_m = float(log_altitude_m) if log_altitude_m is not None else config_altitude_m
    obstacle_map = build_obstacle_map(config, flight_altitude_m=altitude_m)
    obstacles = obstacle_map["inflated_blocking_cells"]
    resolution_m, _ = get_resolution_altitude(config)
    return config_path, config, obstacles, resolution_m, obstacle_map


def obstacle_cell_names(config):
    if not config:
        return {}
    width = int(config["width"])
    height = int(config["height"])
    names_by_cell = {}
    for obstacle in config.get("obstacles", []):
        name = obstacle.get("name", "<unnamed>")
        if obstacle.get("type") == "rect":
            cells = [
                (x, y)
                for x in range(int(obstacle["x_min"]), int(obstacle["x_max"]) + 1)
                for y in range(int(obstacle["y_min"]), int(obstacle["y_max"]) + 1)
            ]
        elif obstacle.get("type") == "cell":
            cells = [(int(obstacle["x"]), int(obstacle["y"]))]
        else:
            cells = []
        for cell in cells:
            x, y = cell
            if 0 <= x < width and 0 <= y < height:
                names_by_cell.setdefault(cell, []).append(name)
    return names_by_cell


def wp_number(target_name):
    match = re.search(r"(\d+)$", str(target_name))
    if not match:
        return None
    return int(match.group(1))


def is_wp_target(target_name):
    return re.match(r"^WP\d+$", str(target_name)) is not None


def is_replanned_wp_target(target_name):
    return re.match(r"^RWP\d+$", str(target_name)) is not None


def active_replan_replacement_events(df):
    required = {"elapsed_s", "target_name", "replan_mode", "replan_route_replaced"}
    if not required.issubset(df.columns):
        return []

    events = []
    previous_name = None
    previous_non_empty_name = None
    previous_route = None
    for _, row in df.dropna(subset=["elapsed_s"]).iterrows():
        name = str(row.get("target_name", "")).strip()
        if name and name != previous_name:
            previous_non_empty_name = previous_name if previous_name else previous_non_empty_name
            previous_name = name
        route = str(row.get("route_direction", "")).strip()
        if route:
            previous_route = route

        replan_mode = str(row.get("replan_mode", "")).strip().lower()
        route_replaced = bool_from_value(row.get("replan_route_replaced"))
        if replan_mode != "active" or not route_replaced:
            continue

        replacement_time = float(row["elapsed_s"])
        target_before = previous_non_empty_name if is_replanned_wp_target(name) else name
        target_before = target_before or previous_non_empty_name
        after_rows = df[
            (df["elapsed_s"] >= replacement_time)
            & (df["target_name"].astype(str).str.strip() != "")
        ]
        replanned_rows = after_rows[
            after_rows["target_name"].astype(str).str.strip().map(is_replanned_wp_target)
        ]
        first_replanned_target = None
        first_replanned_time = None
        if not replanned_rows.empty:
            first_replanned = replanned_rows.iloc[0]
            first_replanned_target = str(first_replanned["target_name"]).strip()
            first_replanned_time = float(first_replanned["elapsed_s"])

        events.append(
            {
                "replacement_time_s": replacement_time,
                "target_before_replacement": target_before,
                "first_replanned_target": first_replanned_target,
                "first_replanned_target_time_s": first_replanned_time,
                "active_replan_path_length": safe_last_valid(row.to_frame().T, "active_replan_path_length"),
                "route_direction": previous_route,
            }
        )
    return events


def expected_active_replan_transition(previous_name, name, elapsed_s, events):
    if not is_wp_target(previous_name) or not is_replanned_wp_target(name):
        return False
    for event in events:
        if event.get("target_before_replacement") != previous_name:
            continue
        if event.get("first_replanned_target") != name:
            continue
        event_time = event.get("replacement_time_s")
        target_time = event.get("first_replanned_target_time_s")
        if event_time is None:
            continue
        if target_time is not None and abs(float(target_time) - float(elapsed_s)) <= 1.0:
            return True
        if 0 <= float(elapsed_s) - float(event_time) <= 2.0:
            return True
    return False


def compute_warnings(df, obstacles, resolution_m, obstacle_config_path, collision_report):
    warnings = []
    position_df = df[["elapsed_s", "local_north_m", "local_east_m"]].dropna()
    jumps = []
    previous = None
    for _, row in position_df.iterrows():
        if previous is not None:
            jump = math.hypot(
                float(row["local_east_m"]) - float(previous["local_east_m"]),
                float(row["local_north_m"]) - float(previous["local_north_m"]),
            )
            if jump > PLOT_SPLIT_DISTANCE_M:
                jumps.append((float(previous["elapsed_s"]), float(row["elapsed_s"]), jump))
        previous = row
    if jumps:
        warnings.append(
            f"{len(jumps)} logged position segment(s) are longer than {PLOT_SPLIT_DISTANCE_M:.1f} m; "
            "plots split these segments to avoid misleading diagonal lines."
        )

    if "target_name" in df.columns:
        active_replan_events = active_replan_replacement_events(df)
        target_changes = df[["elapsed_s", "target_name", "route_direction"]].dropna(subset=["elapsed_s"])
        previous_name = None
        previous_num = None
        previous_route = None
        for _, row in target_changes.iterrows():
            name = str(row["target_name"])
            if not name or name == previous_name:
                continue
            num = wp_number(name)
            route = str(row["route_direction"]) if "route_direction" in target_changes.columns else ""
            # A phase boundary may intentionally restore an original waypoint
            # target (for example RWP06 -> WP09 at goal_hover). Target sequence
            # validation is route-scoped, so do not compare numbering across a
            # route-direction boundary.
            if previous_route is not None and route != previous_route:
                previous_name = name
                previous_num = num
                previous_route = route
                continue
            if previous_num is not None and num is not None:
                expected_delta = -1 if route == "return" or previous_route == "return" else 1
                if num - previous_num not in {expected_delta, 0} and abs(num - previous_num) > 1:
                    if expected_active_replan_transition(
                        previous_name,
                        name,
                        float(row["elapsed_s"]),
                        active_replan_events,
                    ):
                        previous_name = name
                        previous_num = num
                        previous_route = route
                        continue
                    warnings.append(
                        f"Target jumped from {previous_name} to {name} at t={float(row['elapsed_s']):.2f}s; "
                        "check target switching."
                    )
                    break
            previous_name = name
            previous_num = num
            previous_route = route

    first_error = safe_last_valid(df.head(1), "horizontal_error_m")
    targets = target_sequence(df, route_direction="outbound") or target_sequence(df)
    if first_error is not None and len(targets) >= 2:
        full_distance = math.hypot(
            targets[-1]["east_m"] - targets[0]["east_m"],
            targets[-1]["north_m"] - targets[0]["north_m"],
        )
        if full_distance > 0 and first_error > full_distance * 0.75:
            warnings.append(
                "First horizontal error is close to the full start-goal distance; "
                "the initial target may be wrong."
            )

    if collision_report["raw_physical_collision_detected"]:
        warnings.append(
            "Actual trajectory entered raw physical footprint cells; this is more serious than a buffer entry."
        )
    if collision_report["inflated_safety_buffer_entry_detected"]:
        warnings.append(
            "Actual trajectory entered inflated planning obstacle cells; this is a safety-buffer violation."
        )
    elif (
        collision_report["approximate_min_clearance_m"] is not None
        and collision_report["approximate_min_clearance_m"] < 1.0
    ):
        warnings.append(
            "Near-boundary clearance warning: actual trajectory came within about "
            f"{collision_report['approximate_min_clearance_m']:.2f} m of an obstacle cell center."
        )

    return warnings, jumps, collision_report


def waypoint_transition_summary(df):
    required = {"target_name", "elapsed_s", "horizontal_error_m"}
    if not required.issubset(df.columns):
        return []

    rows = []
    grouped = df[df["target_name"].astype(str) != ""].groupby(
        ["route_direction", "target_name"] if "route_direction" in df.columns else ["target_name"],
        dropna=False,
        sort=False,
    )
    for key, group in grouped:
        if isinstance(key, tuple):
            route_direction, target_name = key
        else:
            route_direction, target_name = "", key
        errors = group["horizontal_error_m"].dropna()
        reached_rows = group[group["horizontal_error_m"] <= WAYPOINT_REACHED_THRESHOLD_M]
        rows.append(
            {
                "route_direction": route_direction or "none",
                "target_name": target_name,
                "first_time_s": float(group["elapsed_s"].min()),
                "last_time_s": float(group["elapsed_s"].max()),
                "min_horizontal_error_m": None if errors.empty else float(errors.min()),
                "reached": not reached_rows.empty,
                "reached_time_s": None if reached_rows.empty else float(reached_rows["elapsed_s"].iloc[0]),
            }
        )
    return rows


def replan_summary(df):
    if "replan_triggered" not in df.columns:
        return None

    triggered = df["replan_triggered"].map(bool_from_value).fillna(False)
    if "replan_success" in df.columns:
        success = df["replan_success"].map(bool_from_value).fillna(False)
    else:
        success = pd.Series(False, index=df.index)

    total_attempts = int(triggered.sum())
    if "replan_count" in df.columns:
        max_count = df["replan_count"].max(skipna=True)
        if not pd.isna(max_count):
            total_attempts = max(total_attempts, int(max_count))

    route_replaced = pd.Series(False, index=df.index)
    active_replacement_count = None
    max_active_path_length = None
    replan_mode = first_value(df, "replan_mode") or "log_only"
    if "replan_route_replaced" in df.columns:
        route_replaced = df["replan_route_replaced"].map(bool_from_value).fillna(False)
        active_replacement_count = int(route_replaced.sum())
    if "active_replan_count" in df.columns:
        max_active_count = safe_max(df, "active_replan_count")
        if max_active_count is not None:
            active_replacement_count = max(active_replacement_count or 0, int(max_active_count))
    if "active_replan_path_length" in df.columns and route_replaced.any():
        max_active_path_length = safe_max(df[route_replaced], "active_replan_path_length")

    return {
        "available": True,
        "replan_mode": replan_mode,
        "total_replan_attempts": total_attempts,
        "successful_replan_attempts": int((triggered & success).sum()),
        "max_replan_path_length": safe_max(df[triggered], "replan_path_length") if triggered.any() else None,
        "max_dynamic_blocked_cell_count": (
            safe_max(df[triggered], "dynamic_blocked_cell_count") if triggered.any() else None
        ),
        "active_route_replacement_count": active_replacement_count,
        "max_active_replan_path_length": max_active_path_length,
    }


def active_replan_route_replacement_summary(df):
    if "replan_mode" not in df.columns and "replan_route_replaced" not in df.columns:
        return None

    events = active_replan_replacement_events(df)
    replan_mode = first_value(df, "replan_mode") or "log_only"
    route_replaced = pd.Series(False, index=df.index)
    if "replan_route_replaced" in df.columns:
        route_replaced = df["replan_route_replaced"].map(bool_from_value).fillna(False)

    replacement_count = int(route_replaced.sum())
    if "active_replan_count" in df.columns:
        max_active_count = safe_max(df, "active_replan_count")
        if max_active_count is not None:
            replacement_count = max(replacement_count, int(max_active_count))
    replacement_count = max(replacement_count, len(events))

    first_event = events[0] if events else {}
    active_path_length = first_event.get("active_replan_path_length")
    if active_path_length is None and "active_replan_path_length" in df.columns and route_replaced.any():
        active_path_length = safe_last_valid(df[route_replaced].head(1), "active_replan_path_length")
    if active_path_length is None and "active_replan_path_length" in df.columns:
        active_path_length = safe_max(df, "active_replan_path_length")

    return {
        "available": True,
        "replan_mode": replan_mode,
        "active_route_replacement_count": replacement_count,
        "first_active_route_replacement_time_s": first_event.get("replacement_time_s"),
        "target_before_replacement": first_event.get("target_before_replacement"),
        "first_replanned_target_after_replacement": first_event.get("first_replanned_target"),
        "active_replan_path_length": active_path_length,
        "max_active_replan_path_length": safe_max(df, "active_replan_path_length")
        if "active_replan_path_length" in df.columns
        else None,
    }


def perception_summary(df):
    summary = {
        "available": has_perception_columns(df),
        "perception_enabled": False,
        "total_duration_s": duration_s(df),
        "detection_range_m": None,
        "detection_fov_deg": None,
        "warning_distance_m": None,
        "danger_distance_m": None,
        "risk_action": None,
        "total_detection_samples": 0,
        "samples_with_detections": 0,
        "clear_sample_count": 0,
        "detected_sample_count": 0,
        "warning_sample_count": 0,
        "danger_sample_count": 0,
        "clear_sample_ratio": None,
        "detected_sample_ratio": None,
        "warning_sample_ratio": None,
        "danger_sample_ratio": None,
        "time_in_clear_s": 0.0,
        "time_in_detected_s": 0.0,
        "time_in_warning_s": 0.0,
        "time_in_danger_s": 0.0,
        "percent_time_clear": None,
        "percent_time_detected": None,
        "percent_time_warning": None,
        "percent_time_danger": None,
        "clear_samples": 0,
        "detected_samples": 0,
        "warning_samples": 0,
        "danger_samples": 0,
        "first_detection_time_s": None,
        "first_warning_time_s": None,
        "first_danger_time_s": None,
        "nearest_obstacle_ever_detected": None,
        "minimum_nearest_obstacle_distance_m": None,
        "mean_nearest_obstacle_distance_m": None,
        "median_nearest_obstacle_distance_m": None,
        "most_frequent_obstacle_names": [],
        "most_frequent_warning_obstacle_names": [],
        "most_frequent_danger_obstacle_names": [],
    }
    if not summary["available"]:
        return summary

    enabled_mask = perception_enabled_mask(df)
    detected_mask = detected_obstacle_mask(df)
    active_df = df[enabled_mask]
    detected_df = df[enabled_mask & detected_mask]

    summary["perception_enabled"] = bool(enabled_mask.any())
    summary["total_detection_samples"] = int(len(active_df))
    summary["samples_with_detections"] = int(len(detected_df))
    detection_range = first_value(active_df, "detection_range_m")
    detection_fov = first_value(active_df, "detection_fov_deg")
    summary["detection_range_m"] = None if detection_range is None else float(detection_range)
    summary["detection_fov_deg"] = None if detection_fov is None else float(detection_fov)
    warning_distance = first_value(active_df, "warning_distance_m")
    danger_distance = first_value(active_df, "danger_distance_m")
    summary["warning_distance_m"] = (
        DEFAULT_WARNING_DISTANCE_M if warning_distance is None else float(warning_distance)
    )
    summary["danger_distance_m"] = (
        DEFAULT_DANGER_DISTANCE_M if danger_distance is None else float(danger_distance)
    )
    summary["risk_action"] = first_value(active_df, "risk_action") or "log_only"

    risk_levels = risk_level_series(active_df)
    summary["clear_sample_count"] = int((risk_levels == "clear").sum())
    summary["detected_sample_count"] = int((risk_levels == "detected").sum())
    summary["warning_sample_count"] = int((risk_levels == "warning").sum())
    summary["danger_sample_count"] = int((risk_levels == "danger").sum())
    summary["clear_samples"] = summary["clear_sample_count"]
    summary["detected_samples"] = summary["detected_sample_count"]
    summary["warning_samples"] = summary["warning_sample_count"]
    summary["danger_samples"] = summary["danger_sample_count"]
    total_samples = summary["total_detection_samples"]
    summary["clear_sample_ratio"] = ratio(summary["clear_sample_count"], total_samples)
    summary["detected_sample_ratio"] = ratio(summary["detected_sample_count"], total_samples)
    summary["warning_sample_ratio"] = ratio(summary["warning_sample_count"], total_samples)
    summary["danger_sample_ratio"] = ratio(summary["danger_sample_count"], total_samples)

    risk_time = time_in_risk_levels(active_df, risk_levels)
    for level in RISK_LEVEL_TO_VALUE:
        summary[f"time_in_{level}_s"] = risk_time[level]
    risk_time_total = sum(risk_time.values())
    for level in RISK_LEVEL_TO_VALUE:
        value = ratio(risk_time[level], risk_time_total)
        summary[f"percent_time_{level}"] = None if value is None else value * 100.0

    warning_df = active_df[risk_levels == "warning"]
    danger_df = active_df[risk_levels == "danger"]
    if not warning_df.empty:
        summary["first_warning_time_s"] = safe_last_valid(warning_df.head(1), "elapsed_s")
    if not danger_df.empty:
        summary["first_danger_time_s"] = safe_last_valid(danger_df.head(1), "elapsed_s")

    if detected_df.empty:
        return summary

    summary["first_detection_time_s"] = safe_last_valid(detected_df.head(1), "elapsed_s")
    distances = detected_df["nearest_obstacle_distance_m"].dropna()
    if not distances.empty:
        min_index = distances.idxmin()
        summary["minimum_nearest_obstacle_distance_m"] = float(distances.loc[min_index])
        summary["mean_nearest_obstacle_distance_m"] = float(distances.mean())
        summary["median_nearest_obstacle_distance_m"] = float(distances.median())
        summary["nearest_obstacle_ever_detected"] = str(
            detected_df.loc[min_index].get("nearest_obstacle_name", "")
        )

    summary["most_frequent_obstacle_names"] = frequent_obstacle_names(detected_df)
    summary["most_frequent_warning_obstacle_names"] = frequent_obstacle_names(warning_df)
    summary["most_frequent_danger_obstacle_names"] = frequent_obstacle_names(danger_df)
    return summary


def cleanup_old_astar_outputs():
    return []


def load_run_status(log_path, warnings):
    status_path = log_path.with_suffix(".status.json")
    if not status_path.exists():
        return None
    try:
        return json.loads(status_path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        warnings.append(f"Could not read run status {display_path(status_path)}: {error}")
        return None


def main():
    args = parse_args()
    ensure_output_tree()
    try:
        log_path, used_newest_log = resolve_log_path(args.log)
    except FileNotFoundError as error:
        print(f"Analysis skipped: {error}")
        print("Run an A* flight first, or pass --log path/to/astar_*.csv.")
        return 2

    run_id = run_id_from_log(log_path)
    df = prepare_dataframe(log_path)
    stage = infer_experiment_stage(df)
    experiment_type = infer_experiment_type(df, stage)
    output_dir = get_run_output_dir(stage, run_id)
    output_dir.mkdir(parents=True, exist_ok=True)
    created_at_utc = datetime.now(timezone.utc).isoformat()
    initial_warnings = []
    run_status = load_run_status(log_path, initial_warnings)
    if run_status and (
        run_status.get("status") != "completed"
        or run_status.get("landing_confirmed") is not True
    ):
        initial_warnings.append(
            "Flight runtime status is not a confirmed completion: "
            f"status={run_status.get('status')}, "
            f"landing_confirmed={run_status.get('landing_confirmed')}, "
            f"message={run_status.get('message') or 'N/A'}"
        )
    obstacle_config_path, obstacle_config, obstacles, obstacle_resolution_m, obstacle_map = load_analysis_obstacles(
        args, df, initial_warnings
    )
    if obstacle_config and (first_value(df, "map_name") is None):
        df["map_name"] = obstacle_config.get("map_name", "")
    log_resolution = safe_last_valid(df, "resolution_m")
    resolution_m = obstacle_resolution_m or log_resolution or 1.0
    collision_report = obstacle_collision_report(
        df,
        obstacle_map,
        resolution_m,
    )
    warnings, jumps, collision_report = compute_warnings(
        df,
        obstacles,
        resolution_m,
        obstacle_config_path,
        collision_report,
    )
    warnings = [*initial_warnings, *warnings]

    print(f"Analyzing log: {display_path(log_path)}")
    print(f"Run ID: {run_id}")
    print(f"Output stage: {stage}")
    print(f"Rows: {len(df)}")
    if obstacle_map:
        print("Height-aware analysis:")
        print(f"  flight altitude: {obstacle_map['altitude_m']} m")
        print(f"  vertical safety margin: {obstacle_map['vertical_safety_margin_m']} m")
        print(f"  horizontal inflation cells: {obstacle_map['horizontal_inflation_cells']}")
        print(f"  blocking obstacles: {obstacle_map['blocking_obstacle_names']}")
        print(f"  ignored/nonblocking obstacles: {obstacle_map['nonblocking_obstacle_names']}")
    if warnings:
        print("Analysis warnings:")
        for warning in warnings:
            print(f"  WARNING: {warning}")

    core_generated_files = []
    debug_generated_files = []
    collision_generated_files = []
    source_log = str(display_path(log_path))

    if args.debug_plots:
        plot_path = save_line_plot(
            df,
            ["altitude_m"],
            "Altitude Over Time",
            "Altitude above local origin (m)",
            output_dir / "alt.png",
            source_log,
        )
        if plot_path is not None:
            debug_generated_files.append(plot_path)

    map_name = first_value(df, "map_name")
    plot_path = save_trajectory_plot(
        df,
        output_dir / "traj.png",
        log_path,
        map_name,
        obstacle_map,
        resolution_m,
        collision_report,
        title="A* Local 2D Trajectory",
        show_obstacles=False,
        show_collision_points=False,
    )
    if plot_path is not None:
        core_generated_files.append(plot_path)

    if obstacle_config_path is not None and obstacle_map:
        plot_path = save_trajectory_plot(
            df,
            output_dir / "traj_with_obstacles.png",
            log_path,
            map_name,
            obstacle_map,
            resolution_m,
            collision_report,
            title="A* Local 2D Trajectory with Obstacle Validation",
            show_obstacles=True,
            show_collision_points=True,
            show_perception_points=True,
        )
        if plot_path is not None:
            core_generated_files.append(plot_path)
    else:
        print(
            "Skipping traj_with_obstacles.png: pass --obstacle-config to enable the obstacle validation overlay."
        )

    plot_path = save_error_plot(df, output_dir / "error.png", source_log)
    if plot_path is not None:
        core_generated_files.append(plot_path)

    if args.debug_plots:
        plot_path = save_line_plot(
            df,
            ["velocity_north_m_s", "velocity_east_m_s", "velocity_down_m_s"],
            "NED Velocity Over Time",
            "Velocity (m/s)",
            output_dir / "vel.png",
            source_log,
        )
        if plot_path is not None:
            debug_generated_files.append(plot_path)

        plot_path = save_line_plot(
            df,
            ["yaw_deg"],
            "Yaw Over Time",
            "Yaw (deg)",
            output_dir / "yaw.png",
            source_log,
        )
        if plot_path is not None:
            debug_generated_files.append(plot_path)

        plot_path = save_target_timeline(df, output_dir / "target_timeline.png", source_log)
        if plot_path is not None:
            debug_generated_files.append(plot_path)

    if has_perception_columns(df):
        plot_path = save_perception_risk_timeline(
            df,
            output_dir / "perception_risk_timeline.png",
            source_log,
        )
        if plot_path is not None:
            core_generated_files.append(plot_path)

        if args.debug_plots:
            plot_path = save_perception_timeline(
                df,
                output_dir / "perception_timeline.png",
                source_log,
            )
            if plot_path is not None:
                debug_generated_files.append(plot_path)

            plot_path = save_detection_count_plot(
                df,
                output_dir / "detection_count_over_time.png",
                source_log,
            )
            if plot_path is not None:
                debug_generated_files.append(plot_path)

    if collision_report["obstacle_collision_detected"]:
        collision_csv = save_collision_points_csv(
            collision_report,
            output_dir / "collision_points.csv",
        )
        if collision_csv is not None:
            collision_generated_files.append(collision_csv)

        collision_zoom = save_collision_zoom_plot(
            df,
            output_dir / "collision_zoom.png",
            obstacle_map,
            resolution_m,
            collision_report,
        )
        if collision_zoom is not None:
            collision_generated_files.append(collision_zoom)
    else:
        print("No collision/buffer entries detected; collision debug files were not generated.")

    summary_path = output_dir / "summary.md"
    manifest_path = output_dir / "manifest.json"
    metadata_path = output_dir / "run_metadata.json"
    core_generated_files = [summary_path, manifest_path, metadata_path, *core_generated_files]
    summary_path = write_summary(
        output_dir=output_dir,
        log_path=log_path,
        run_id=run_id,
        created_at_utc=created_at_utc,
        df=df,
        core_generated_files=core_generated_files,
        debug_generated_files=debug_generated_files,
        collision_generated_files=collision_generated_files,
        debug_plots_enabled=args.debug_plots,
        obstacle_config_path=obstacle_config_path,
        collision_report=collision_report,
        warnings=warnings,
        obstacle_map=obstacle_map,
        waypoint_transition_summary_func=waypoint_transition_summary,
        perception_summary_func=perception_summary,
        replan_summary_func=replan_summary,
        active_replan_route_replacement_summary_func=active_replan_route_replacement_summary,
        active_replan_target_validation_func=lambda frame: validate_active_replan_rows(
            frame.to_dict(orient="records")
        ),
        infer_return_home_enabled_func=infer_return_home_enabled,
        waypoint_reached_threshold_m=WAYPOINT_REACHED_THRESHOLD_M,
        run_status=run_status,
    )
    manifest_path = write_manifest(
        output_dir=output_dir,
        log_path=log_path,
        run_id=run_id,
        created_at_utc=created_at_utc,
        df=df,
        core_generated_files=core_generated_files,
        debug_generated_files=debug_generated_files,
        collision_generated_files=collision_generated_files,
        debug_plots_enabled=args.debug_plots,
        obstacle_config_path=obstacle_config_path,
        warnings=warnings,
        obstacle_map=obstacle_map,
        collision_report=collision_report,
        perception_summary_func=perception_summary,
        replan_summary_func=replan_summary,
        active_replan_route_replacement_summary_func=active_replan_route_replacement_summary,
        active_replan_target_validation_func=lambda frame: validate_active_replan_rows(
            frame.to_dict(orient="records")
        ),
        run_status=run_status,
    )
    metadata_path = write_run_metadata(
        output_dir=output_dir,
        run_id=run_id,
        stage=stage,
        experiment_type=experiment_type,
        log_path=log_path,
        obstacle_config_path=obstacle_config_path,
        df=df,
        infer_local_replan_enabled_func=infer_local_replan_enabled,
        infer_return_home_enabled_func=infer_return_home_enabled,
        run_status=run_status,
    )
    generated_files = [
        *core_generated_files,
        *debug_generated_files,
        *collision_generated_files,
    ]

    output_dir.touch()
    cleanup_old_astar_outputs()

    print("\nAnalysis complete.")
    print(f"Source log: {display_path(log_path)}")
    print(f"Output folder: {display_path(output_dir)}")
    print("Generated files:")
    for path in generated_files:
        print(f"* {display_path(path)}")

    if used_newest_log:
        print("\nTo analyze this exact run again:")
        command = f"python main.py report analyze --log {display_path(log_path)}"
        if obstacle_config_path:
            command += f" --obstacle-config {display_path(obstacle_config_path)}"
        print(command)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
