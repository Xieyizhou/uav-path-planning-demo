import csv
import json
import math
import re
from datetime import datetime, timezone
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
import sys

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.logging.evaluation_schema import EVALUATION_COLUMNS
from src.logging.output_registry import (
    OUTPUT_ROOT,
    RUN_STAGES,
    STAGE_DIR_NAMES,
    STAGE_LABELS,
    display_path,
    ensure_output_tree,
    get_comparisons_dir,
    get_runs_dir,
    get_summary_paths,
)

# A final position error below this value is treated as good enough for a
# high-level benchmark pass. The low-level waypoint acceptance threshold is
# still recorded by the flight analysis summaries.
FINAL_ERROR_PASS_THRESHOLD_M = 1.0

SUMMARY_COLUMNS = [
    "status",
    "run_id",
    "source_log",
    "risk_action",
    "map_name",
    "altitude_m",
    "max_speed_m_s",
    "speed_source",
    "return_home_enabled",
    "duration_s",
    "warning_sample_ratio",
    "danger_sample_ratio",
    "percent_time_warning",
    "percent_time_danger",
    "minimum_nearest_obstacle_distance_m",
    "mean_nearest_obstacle_distance_m",
    "median_nearest_obstacle_distance_m",
    "max_horizontal_error_m",
    "mean_horizontal_error_m",
    "final_horizontal_error_m",
    "approximate_min_clearance_m",
    "raw_physical_collision_detected",
    "inflated_safety_buffer_entry_detected",
    "obstacle_collision_detected",
    "active_route_replacement_count",
    "max_active_replan_path_length",
    "waypoints_reached",
]


def load_json(path):
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError as error:
        print(f"WARNING: Could not parse {path}: {error}")
        return {}


def read_text(path):
    if not path.exists():
        return ""
    return path.read_text(errors="replace")


def clean_text(value):
    if value is None:
        return None
    return str(value).strip().strip("`")


def first_non_empty(*values):
    for value in values:
        if value is None:
            continue
        if isinstance(value, str) and value.strip() == "":
            continue
        return value
    return None


def parse_float(value):
    if value is None or value == "":
        return None
    if isinstance(value, (int, float)):
        if math.isnan(float(value)):
            return None
        return float(value)
    match = re.search(r"-?\d+(?:\.\d+)?", str(value))
    if not match:
        return None
    return float(match.group(0))


def parse_bool(value):
    if value is None or value == "":
        return None
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().strip("`").lower()
    if normalized in {"true", "yes", "1"}:
        return True
    if normalized in {"false", "no", "0"}:
        return False
    return None


def format_float(value):
    value = parse_float(value)
    if value is None:
        return ""
    return f"{value:.3f}"


def format_bool(value):
    parsed = parse_bool(value)
    if parsed is None:
        return ""
    return "yes" if parsed else "no"


def parse_summary_bullets(summary_text):
    values = {}
    for line in summary_text.splitlines():
        match = re.match(r"^- ([^:]+):\s*(.*)$", line.strip())
        if not match:
            continue
        key = match.group(1).strip().lower().replace("-", "_").replace(" ", "_")
        values[key] = clean_text(match.group(2))
    return values


def waypoint_reached_count(summary_text):
    count = 0
    for line in summary_text.splitlines():
        if not line.startswith("|"):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) < 6:
            continue
        if cells[0] in {"route_direction", "---"}:
            continue
        if cells[5].lower() == "yes":
            count += 1
    return count if count else None


def resolve_project_path(path_text):
    if not path_text:
        return None
    path = Path(str(path_text))
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def csv_rows(source_log):
    path = resolve_project_path(source_log)
    if path is None or not path.exists():
        return []
    try:
        with path.open(newline="") as csv_file:
            return list(csv.DictReader(csv_file))
    except OSError as error:
        print(f"WARNING: Could not read {path}: {error}")
        return []


def first_csv_value(rows, column):
    for row in rows:
        value = row.get(column)
        if value not in {None, ""}:
            return value
    return None


def last_csv_float(rows, column):
    for row in reversed(rows):
        value = parse_float(row.get(column))
        if value is not None:
            return value
    return None


def max_csv_float(rows, column):
    values = [parse_float(row.get(column)) for row in rows]
    values = [value for value in values if value is not None]
    return max(values) if values else None


def count_csv_true(rows, column):
    if not rows or not any(column in row for row in rows):
        return None
    return sum(1 for row in rows if parse_bool(row.get(column)))


def mean_csv_float(rows, column):
    values = [parse_float(row.get(column)) for row in rows]
    values = [value for value in values if value is not None]
    return sum(values) / len(values) if values else None


def warning_note(warnings):
    if not warnings:
        return ""
    if isinstance(warnings, list):
        return "; ".join(str(warning) for warning in warnings if warning)
    return str(warnings)


def target_points(rows):
    points = []
    previous_key = None
    for row in rows:
        name = str(row.get("target_name") or "").strip()
        north = parse_float(row.get("target_north_m"))
        east = parse_float(row.get("target_east_m"))
        if not name or north is None or east is None:
            continue
        key = (row.get("route_direction") or "", name, north, east)
        if key == previous_key:
            continue
        points.append((north, east))
        previous_key = key
    return points


def path_length_from_points(points):
    total = 0.0
    previous = None
    for point in points:
        if previous is not None:
            total += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
    return total if total > 0 else None


def planned_path_length(rows):
    return path_length_from_points(target_points(rows))


def approximate_distance_traveled(rows):
    total = 0.0
    previous = None
    for row in rows:
        north = parse_float(row.get("local_north_m"))
        east = parse_float(row.get("local_east_m"))
        if north is None or east is None:
            continue
        current = (north, east)
        if previous is not None:
            step = math.hypot(current[0] - previous[0], current[1] - previous[1])
            if step <= 3.0:
                total += step
        previous = current
    return total if total > 0 else None


def count_risk_rows(rows, risk_levels=None):
    if not rows or not any("perception_risk_level" in row for row in rows):
        return None
    risk_levels = set(risk_levels or {"detected", "warning", "danger"})
    return sum(
        1
        for row in rows
        if str(row.get("perception_risk_level") or "").strip().lower() in risk_levels
    )


def slow_down_event_count(rows, risk_action):
    if risk_action != "slow_down":
        return 0 if risk_action else None
    count = count_risk_rows(rows, {"warning", "danger"})
    return count


def completed_or_failed(status):
    if status == "FAIL":
        return "failed"
    if status == "PASS":
        return "completed"
    return "needs_review"


def measured_max_horizontal_speed(rows):
    speeds = []
    for row in rows:
        north = parse_float(row.get("velocity_north_m_s"))
        east = parse_float(row.get("velocity_east_m_s"))
        if north is None or east is None:
            continue
        speeds.append(math.hypot(north, east))
    return max(speeds) if speeds else None


def commanded_speed(rows):
    speed_columns = [
        "commanded_speed_m_s",
        "commanded_max_speed_m_s",
        "max_speed_m_s",
        "max_horizontal_speed_m_s",
    ]
    for column in speed_columns:
        value = max_csv_float(rows, column)
        if value is not None:
            return value, column
    return None, None


def choose_status(raw_collision, inflated_entry, final_error):
    if parse_bool(raw_collision):
        return "FAIL"
    if parse_bool(inflated_entry):
        return "WARNING"
    final_error = parse_float(final_error)
    if final_error is None or final_error > FINAL_ERROR_PASS_THRESHOLD_M:
        return "WARNING"
    return "PASS"


def collect_run(output_dir):
    manifest = load_json(output_dir / "manifest.json")
    metadata = load_json(output_dir / "run_metadata.json")
    summary_text = read_text(output_dir / "summary.md")
    summary = parse_summary_bullets(summary_text)
    collision = manifest.get("collision_report", {})
    height = manifest.get("height_aware_planning", {})
    perception = manifest.get("perception_summary", {})
    replan = manifest.get("replan_summary", {}) or {}
    active_replan = manifest.get("active_replan_route_replacement_summary", {}) or {}

    run_id = first_non_empty(manifest.get("run_id"), summary.get("run_id"), output_dir.name)
    source_log = first_non_empty(manifest.get("source_log"), summary.get("source_log"))
    rows = csv_rows(source_log)

    speed, speed_source = commanded_speed(rows)
    if speed is None:
        speed = measured_max_horizontal_speed(rows)
        speed_source = "measured_velocity"

    raw_collision = first_non_empty(
        collision.get("raw_physical_collision_detected"),
        summary.get("raw_physical_collision_detected"),
    )
    inflated_entry = first_non_empty(
        collision.get("inflated_safety_buffer_entry_detected"),
        summary.get("inflated_safety_buffer_entry_detected"),
    )
    obstacle_collision = first_non_empty(
        collision.get("obstacle_collision_detected"),
        summary.get("obstacle_collision_detected"),
    )
    if obstacle_collision is None:
        obstacle_collision = bool(parse_bool(raw_collision) or parse_bool(inflated_entry))

    final_error = first_non_empty(
        summary.get("final_horizontal_error"),
        last_csv_float(rows, "horizontal_error_m"),
    )

    inferred_stage = first_non_empty(metadata.get("stage"), infer_stage_from_output_dir(output_dir))
    result = {
        "stage": inferred_stage,
        "stage_folder": STAGE_DIR_NAMES.get(inferred_stage),
        "experiment_type": metadata.get("experiment_type"),
        "run_id": run_id,
        "source_log": display_path(source_log),
        "risk_action": first_non_empty(
            perception.get("risk_action"),
            summary.get("risk_action"),
            first_csv_value(rows, "risk_action"),
        ),
        "map_name": first_non_empty(
            manifest.get("map_name"),
            summary.get("map_name"),
            first_csv_value(rows, "map_name"),
        ),
        "altitude_m": first_non_empty(
            height.get("altitude_m"),
            summary.get("height_aware_altitude"),
            summary.get("max_altitude"),
            first_csv_value(rows, "altitude_m"),
        ),
        "max_speed_m_s": speed,
        "speed_source": speed_source,
        "return_home_enabled": first_non_empty(
            summary.get("return_home_enabled"),
            first_csv_value(rows, "return_home_enabled"),
        ),
        "duration_s": first_non_empty(
            manifest.get("duration_s"),
            perception.get("total_duration_s"),
            summary.get("duration"),
            summary.get("total_duration_s"),
            max_csv_float(rows, "elapsed_s"),
        ),
        "warning_sample_ratio": first_non_empty(
            perception.get("warning_sample_ratio"),
            summary.get("warning_sample_ratio"),
        ),
        "danger_sample_ratio": first_non_empty(
            perception.get("danger_sample_ratio"),
            summary.get("danger_sample_ratio"),
        ),
        "percent_time_warning": first_non_empty(
            perception.get("percent_time_warning"),
            summary.get("percent_time_warning"),
        ),
        "percent_time_danger": first_non_empty(
            perception.get("percent_time_danger"),
            summary.get("percent_time_danger"),
        ),
        "minimum_nearest_obstacle_distance_m": first_non_empty(
            perception.get("minimum_nearest_obstacle_distance_m"),
            summary.get("minimum_nearest_obstacle_distance_m"),
            summary.get("minimum_nearest_obstacle_distance"),
        ),
        "mean_nearest_obstacle_distance_m": first_non_empty(
            perception.get("mean_nearest_obstacle_distance_m"),
            summary.get("mean_nearest_obstacle_distance_m"),
        ),
        "median_nearest_obstacle_distance_m": first_non_empty(
            perception.get("median_nearest_obstacle_distance_m"),
            summary.get("median_nearest_obstacle_distance_m"),
        ),
        "max_horizontal_error_m": first_non_empty(
            summary.get("max_horizontal_error"),
            max_csv_float(rows, "horizontal_error_m"),
        ),
        "mean_horizontal_error_m": first_non_empty(
            summary.get("mean_horizontal_error"),
            mean_csv_float(rows, "horizontal_error_m"),
        ),
        "final_horizontal_error_m": final_error,
        "approximate_min_clearance_m": first_non_empty(
            collision.get("approximate_min_clearance_m"),
            summary.get("approximate_min_clearance_m"),
        ),
        "raw_physical_collision_detected": raw_collision,
        "inflated_safety_buffer_entry_detected": inflated_entry,
        "obstacle_collision_detected": obstacle_collision,
        "active_route_replacement_count": first_non_empty(
            active_replan.get("active_route_replacement_count"),
            replan.get("active_route_replacement_count"),
            summary.get("active_route_replacement_count"),
            max_csv_float(rows, "active_replan_count"),
            count_csv_true(rows, "replan_route_replaced"),
        ),
        "total_replan_attempts": first_non_empty(
            replan.get("total_replan_attempts"),
            max_csv_float(rows, "replan_count"),
        ),
        "successful_replan_attempts": first_non_empty(
            replan.get("successful_replan_attempts"),
            count_csv_true(rows, "replan_success"),
        ),
        "safety_buffer_violation_count": first_non_empty(
            collision.get("inflated_buffer_entry_points"),
            summary.get("inflated_buffer_entry_points"),
        ),
        "perception_risk_detection_count": first_non_empty(
            perception.get("samples_with_detections"),
            count_risk_rows(rows),
        ),
        "slow_down_event_count": slow_down_event_count(
            rows,
            first_non_empty(
                perception.get("risk_action"),
                summary.get("risk_action"),
                first_csv_value(rows, "risk_action"),
            ),
        ),
        "planned_path_length_m": planned_path_length(rows),
        "actual_traveled_distance_m": approximate_distance_traveled(rows),
        "notes": warning_note(manifest.get("warnings")),
        "max_active_replan_path_length": first_non_empty(
            active_replan.get("max_active_replan_path_length"),
            replan.get("max_active_replan_path_length"),
            summary.get("max_active_replan_path_length"),
            summary.get("active_replan_path_length"),
            max_csv_float(rows, "active_replan_path_length"),
        ),
        "waypoints_reached": waypoint_reached_count(summary_text),
    }
    result["status"] = choose_status(
        result["raw_physical_collision_detected"],
        result["inflated_safety_buffer_entry_detected"],
        result["final_horizontal_error_m"],
    )
    result["completed_or_failed"] = completed_or_failed(result["status"])
    result["final_status"] = result["status"]
    return result


def infer_stage_from_output_dir(output_dir):
    parts = set(output_dir.parts)
    for stage_name, folder_name in STAGE_DIR_NAMES.items():
        if folder_name in parts:
            return stage_name
    return None


def find_analysis_dirs():
    dirs = []
    for stage_name in RUN_STAGES:
        dirs.extend(find_stage_analysis_dirs(stage_name))
    return dirs


def find_stage_analysis_dirs(stage_name):
    runs_dir = get_runs_dir(stage_name)
    if not runs_dir.exists():
        return []
    dirs = []
    for path in sorted(runs_dir.glob("as_*")):
        if not path.is_dir():
            continue
        if (path / "manifest.json").exists() or (path / "summary.md").exists():
            dirs.append(path)
    return dirs


def normalized_csv_row(row):
    normalized = {}
    for column in SUMMARY_COLUMNS:
        value = row.get(column)
        if (
            column.endswith("_m")
            or column.endswith("_s")
            or column.endswith("_ratio")
            or column.startswith("percent_time_")
            or column == "max_speed_m_s"
            or column == "max_active_replan_path_length"
        ):
            normalized[column] = format_float(value)
        elif column.endswith("_detected") or column == "return_home_enabled":
            normalized[column] = format_bool(value)
        elif column in {"waypoints_reached", "active_route_replacement_count"}:
            normalized[column] = "" if value is None else str(value)
        else:
            normalized[column] = "" if value is None else str(value)
    return normalized


def normalized_evaluation_row(row):
    normalized = {}
    for column in EVALUATION_COLUMNS:
        value = row.get(column)
        if column.endswith("_m") or column.endswith("_s"):
            normalized[column] = format_float(value)
        elif column.endswith("_count"):
            parsed = parse_float(value)
            normalized[column] = "" if parsed is None else str(int(parsed))
        else:
            normalized[column] = "" if value is None else str(value)
    return normalized


def write_csv(rows, summary_csv):
    summary_csv.parent.mkdir(parents=True, exist_ok=True)
    with summary_csv.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(normalized_csv_row(row))


def write_evaluation_csv(rows, evaluation_csv):
    evaluation_csv.parent.mkdir(parents=True, exist_ok=True)
    with evaluation_csv.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=EVALUATION_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(normalized_evaluation_row(row))


def markdown_value(row, column):
    value = normalized_csv_row(row).get(column, "")
    return value if value != "" else "N/A"


def evaluation_markdown_value(row, column):
    value = normalized_evaluation_row(row).get(column, "")
    return value if value != "" else "unavailable"


def write_evaluation_markdown(rows, stage_name, evaluation_csv, evaluation_md):
    created_at = datetime.now(timezone.utc).isoformat()
    stage_label = STAGE_LABELS.get(stage_name, stage_name)
    lines = [
        f"# A* Experiment Evaluation: {stage_label}",
        "",
        f"Generated: `{created_at}`",
        "",
        "This table is the formal evaluation layer for this stage. Missing metrics are shown as `unavailable` instead of estimated.",
        "",
    ]
    if not rows:
        lines.append("No analyzed runs were found for this stage.")
    else:
        lines.extend(
            [
                "| run_id | stage | experiment type | completed/failed | flight time (s) | planned path (m) | actual distance (m) | min obstacle distance (m) | buffer violations | risk detections | slow_down events | replan attempts | successful replans | active replacements | final status | notes |",
                "|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|",
            ]
        )
        for row in rows:
            lines.append(
                "| "
                f"{evaluation_markdown_value(row, 'run_id')} | "
                f"{evaluation_markdown_value(row, 'stage_folder')} | "
                f"{evaluation_markdown_value(row, 'experiment_type')} | "
                f"{evaluation_markdown_value(row, 'completed_or_failed')} | "
                f"{evaluation_markdown_value(row, 'total_flight_time_s')} | "
                f"{evaluation_markdown_value(row, 'planned_path_length_m')} | "
                f"{evaluation_markdown_value(row, 'actual_traveled_distance_m')} | "
                f"{evaluation_markdown_value(row, 'minimum_distance_to_obstacle_m')} | "
                f"{evaluation_markdown_value(row, 'safety_buffer_violation_count')} | "
                f"{evaluation_markdown_value(row, 'perception_risk_detection_count')} | "
                f"{evaluation_markdown_value(row, 'slow_down_event_count')} | "
                f"{evaluation_markdown_value(row, 'local_replan_attempt_count')} | "
                f"{evaluation_markdown_value(row, 'successful_local_replan_count')} | "
                f"{evaluation_markdown_value(row, 'active_route_replacement_count')} | "
                f"{evaluation_markdown_value(row, 'final_status')} | "
                f"{evaluation_markdown_value(row, 'notes')} |"
            )
    lines.extend(["", f"CSV output: `{display_path(evaluation_csv)}`"])
    evaluation_md.write_text("\n".join(lines) + "\n")


def to_evaluation_row(row):
    return {
        "run_id": row.get("run_id"),
        "stage": row.get("stage"),
        "stage_folder": row.get("stage_folder") or STAGE_DIR_NAMES.get(row.get("stage")),
        "experiment_type": row.get("experiment_type"),
        "completed_or_failed": row.get("completed_or_failed"),
        "total_flight_time_s": row.get("duration_s"),
        "planned_path_length_m": row.get("planned_path_length_m"),
        "actual_traveled_distance_m": row.get("actual_traveled_distance_m"),
        "minimum_distance_to_obstacle_m": row.get("minimum_nearest_obstacle_distance_m"),
        "safety_buffer_violation_count": row.get("safety_buffer_violation_count"),
        "perception_risk_detection_count": row.get("perception_risk_detection_count"),
        "slow_down_event_count": row.get("slow_down_event_count"),
        "local_replan_attempt_count": row.get("total_replan_attempts"),
        "successful_local_replan_count": row.get("successful_replan_attempts"),
        "active_route_replacement_count": row.get("active_route_replacement_count"),
        "final_status": row.get("final_status") or row.get("status"),
        "notes": row.get("notes"),
    }


def average_metric(rows, column):
    values = [parse_float(row.get(column)) for row in rows]
    values = [value for value in values if value is not None]
    if not values:
        return None
    return sum(values) / len(values)


def grouped_by_risk_action(rows):
    groups = {}
    for row in rows:
        risk_action = first_non_empty(row.get("risk_action"), "N/A")
        groups.setdefault(str(risk_action), []).append(row)
    return dict(sorted(groups.items(), key=lambda item: item[0]))


def risk_action_group_stats(rows):
    metrics = [
        "warning_sample_ratio",
        "danger_sample_ratio",
        "percent_time_warning",
        "percent_time_danger",
        "minimum_nearest_obstacle_distance_m",
        "mean_nearest_obstacle_distance_m",
        "duration_s",
    ]
    return {
        risk_action: {
            "run_count": len(group_rows),
            **{metric: average_metric(group_rows, metric) for metric in metrics},
        }
        for risk_action, group_rows in grouped_by_risk_action(rows).items()
    }


def format_average(value):
    return "N/A" if value is None else f"{value:.3f}"


def append_risk_action_comparison(lines, rows):
    stats = risk_action_group_stats(rows)
    lines.extend(["", "## Risk Action Comparison", ""])
    if not stats:
        lines.append("No risk_action groups were available.")
        return

    lines.extend(
        [
            "| risk_action | runs | avg warning_sample_ratio | avg danger_sample_ratio | avg percent_time_warning | avg percent_time_danger | avg min obstacle distance (m) | avg mean obstacle distance (m) | avg duration (s) |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for risk_action, values in stats.items():
        lines.append(
            "| "
            f"{risk_action} | "
            f"{values['run_count']} | "
            f"{format_average(values['warning_sample_ratio'])} | "
            f"{format_average(values['danger_sample_ratio'])} | "
            f"{format_average(values['percent_time_warning'])} | "
            f"{format_average(values['percent_time_danger'])} | "
            f"{format_average(values['minimum_nearest_obstacle_distance_m'])} | "
            f"{format_average(values['mean_nearest_obstacle_distance_m'])} | "
            f"{format_average(values['duration_s'])} |"
        )

    log_only = stats.get("log_only")
    slow_down = stats.get("slow_down")
    lines.extend(["", "Interpretation:", ""])
    if not log_only or not slow_down:
        lines.append(
            "- Need both `log_only` and `slow_down` analyzed runs for direct risk-action interpretation."
        )
        return

    slow_danger_ratio = slow_down.get("danger_sample_ratio")
    log_danger_ratio = log_only.get("danger_sample_ratio")
    slow_danger_time = slow_down.get("percent_time_danger")
    log_danger_time = log_only.get("percent_time_danger")
    if (
        slow_danger_ratio is not None
        and log_danger_ratio is not None
        and slow_danger_ratio > log_danger_ratio
    ) or (
        slow_danger_time is not None
        and log_danger_time is not None
        and slow_danger_time > log_danger_time
    ):
        lines.append(
            "- `slow_down` has a higher danger ratio or higher percent_time_danger than `log_only`; slowing down may cause the drone to spend longer near obstacles without changing its path."
        )

    slow_min_distance = slow_down.get("minimum_nearest_obstacle_distance_m")
    log_min_distance = log_only.get("minimum_nearest_obstacle_distance_m")
    clearance_improved = (
        slow_min_distance is not None
        and log_min_distance is not None
        and slow_min_distance > log_min_distance
    )
    if clearance_improved:
        lines.append("- `slow_down` increases minimum obstacle distance, so it may improve clearance.")

    slow_duration = slow_down.get("duration_s")
    log_duration = log_only.get("duration_s")
    if (
        slow_duration is not None
        and log_duration is not None
        and slow_duration > log_duration
        and not clearance_improved
    ):
        lines.append(
            "- `slow_down` increases duration but does not improve minimum clearance; future work should add local replanning or safer waypoint generation."
        )


def write_markdown(rows, stage_name, summary_csv, summary_md):
    created_at = datetime.now(timezone.utc).isoformat()
    stage_label = STAGE_LABELS.get(stage_name, stage_name)
    lines = [
        f"# A* Experiment Benchmark Summary: {stage_label}",
        "",
        f"Generated: `{created_at}`",
        "",
        "Status rules:",
        "",
        f"- `PASS`: no raw physical collision, no inflated buffer entry, and final horizontal error <= {FINAL_ERROR_PASS_THRESHOLD_M:.1f} m.",
        "- `WARNING`: inflated safety buffer was entered, final error is missing/high, or the run needs review.",
        "- `FAIL`: raw physical collision was detected.",
        "",
    ]

    if not rows:
        lines.extend(
            [
                "No analyzed A* runs were found.",
                "",
                "Run `python scripts/analysis/analyze_astar_log.py --obstacle-config config/substation_obstacles.json` first.",
            ]
        )
    else:
        lines.extend(
            [
                "| status | run_id | risk_action | map | alt (m) | speed (m/s) | speed source | return home | duration (s) | warning ratio | danger ratio | % warning | % danger | min obstacle dist (m) | mean obstacle dist (m) | median obstacle dist (m) | raw collision | buffer entry | active replans | max active replan path | waypoints reached |",
                "|---|---|---|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
            ]
        )
        for row in rows:
            lines.append(
                "| "
                f"{markdown_value(row, 'status')} | "
                f"{markdown_value(row, 'run_id')} | "
                f"{markdown_value(row, 'risk_action')} | "
                f"{markdown_value(row, 'map_name')} | "
                f"{markdown_value(row, 'altitude_m')} | "
                f"{markdown_value(row, 'max_speed_m_s')} | "
                f"{markdown_value(row, 'speed_source')} | "
                f"{markdown_value(row, 'return_home_enabled')} | "
                f"{markdown_value(row, 'duration_s')} | "
                f"{markdown_value(row, 'warning_sample_ratio')} | "
                f"{markdown_value(row, 'danger_sample_ratio')} | "
                f"{markdown_value(row, 'percent_time_warning')} | "
                f"{markdown_value(row, 'percent_time_danger')} | "
                f"{markdown_value(row, 'minimum_nearest_obstacle_distance_m')} | "
                f"{markdown_value(row, 'mean_nearest_obstacle_distance_m')} | "
                f"{markdown_value(row, 'median_nearest_obstacle_distance_m')} | "
                f"{markdown_value(row, 'raw_physical_collision_detected')} | "
                f"{markdown_value(row, 'inflated_safety_buffer_entry_detected')} | "
                f"{markdown_value(row, 'active_route_replacement_count')} | "
                f"{markdown_value(row, 'max_active_replan_path_length')} | "
                f"{markdown_value(row, 'waypoints_reached')} |"
            )
        append_risk_action_comparison(lines, rows)

    lines.extend(
        [
            "",
            "CSV output:",
            "",
            f"- `{display_path(summary_csv)}`",
        ]
    )
    summary_md.write_text("\n".join(lines) + "\n")


def write_output_index(stage_counts):
    created_at = datetime.now(timezone.utc).isoformat()
    lines = [
        "# Outputs",
        "",
        f"Generated: `{created_at}`",
        "",
        "New analysis outputs are stage-scoped. Do not write new `as_*` run folders directly under `outputs/`.",
        "",
        "## Stages",
        "",
        "| stage | runs | summaries |",
        "|---|---:|---|",
    ]
    for stage_name in RUN_STAGES:
        paths = get_summary_paths(stage_name)
        summary_links = (
            f"`{display_path(paths['csv'])}`, `{display_path(paths['md'])}`, "
            f"`{display_path(paths['evaluation_csv'])}`, `{display_path(paths['evaluation_md'])}`"
        )
        lines.append(
            f"| `{stage_name}` ({STAGE_LABELS.get(stage_name, stage_name)}) | "
            f"{stage_counts.get(stage_name, 0)} | {summary_links} |"
        )
    lines.extend(
        [
            "",
            "## Comparisons",
            "",
            f"- Cross-stage comparison outputs: `{display_path(get_comparisons_dir())}`",
            "- Use `python scripts/analysis/compare_experiment_sets.py` for intentional landmark comparisons across stages.",
        ]
    )
    (OUTPUT_ROOT / "README.md").write_text("\n".join(lines) + "\n")


def main():
    ensure_output_tree()
    stage_counts = {}
    total_runs = 0
    for stage_name in RUN_STAGES:
        analysis_dirs = find_stage_analysis_dirs(stage_name)
        rows = [collect_run(path) for path in analysis_dirs]
        rows.sort(key=lambda row: row.get("run_id") or "")
        paths = get_summary_paths(stage_name)
        write_csv(rows, paths["csv"])
        write_markdown(rows, stage_name, paths["csv"], paths["md"])
        evaluation_rows = [to_evaluation_row(row) for row in rows]
        write_evaluation_csv(evaluation_rows, paths["evaluation_csv"])
        write_evaluation_markdown(
            evaluation_rows,
            stage_name,
            paths["evaluation_csv"],
            paths["evaluation_md"],
        )
        stage_counts[stage_name] = len(rows)
        total_runs += len(rows)
        print(
            f"{stage_name}: found {len(rows)} analyzed A* run(s); "
            f"wrote {display_path(paths['csv'])}, {display_path(paths['md'])}, "
            f"{display_path(paths['evaluation_csv'])}, and {display_path(paths['evaluation_md'])}"
        )

    write_output_index(stage_counts)
    print(f"Found {total_runs} analyzed A* run(s) across stage summaries.")
    print(f"Wrote {display_path(OUTPUT_ROOT / 'README.md')}")


if __name__ == "__main__":
    main()
