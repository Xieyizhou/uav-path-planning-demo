"""Report and file-writing helpers for offline A* analysis."""

import json

import pandas as pd

from src.logging.log_io import display_path
from src.logging.metrics import (
    duration_s,
    first_active_value,
    first_value,
    format_value,
    perception_enabled_mask,
    safe_last_valid,
    safe_max,
    safe_mean,
)


def save_collision_points_csv(collision_report, output_path):
    rows = collision_report.get("collision_rows", [])
    columns = [
        "entry_type",
        "elapsed_s",
        "local_east_m",
        "local_north_m",
        "grid_x",
        "grid_y",
        "obstacle_names",
    ]
    pd.DataFrame(rows, columns=columns).to_csv(output_path, index=False)
    print(f"Saved collision points: {output_path}")
    return output_path


def markdown_transition_table(rows):
    if not rows:
        return ["No waypoint target data was available."]
    lines = [
        "| route_direction | target_name | first seen (s) | last seen (s) | min error (m) | reached | reached time (s) |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| "
            f"{row['route_direction']} | "
            f"{row['target_name']} | "
            f"{row['first_time_s']:.2f} | "
            f"{row['last_time_s']:.2f} | "
            f"{format_value(row['min_horizontal_error_m'])} | "
            f"{'yes' if row['reached'] else 'no'} | "
            f"{format_value(row['reached_time_s'])} |"
        )
    return lines


def markdown_replan_summary(summary):
    lines = [
        "## Local Replan Summary",
        "",
    ]
    if summary is None:
        lines.append("- No local replan columns were found in this log.")
        return lines

    lines.extend(
        [
            f"- replan_mode: {summary.get('replan_mode') or 'N/A'}",
            f"- total_replan_attempts: {summary['total_replan_attempts']}",
            f"- successful_replan_attempts: {summary['successful_replan_attempts']}",
            f"- max_replan_path_length: {format_value(summary['max_replan_path_length'])}",
            f"- max_dynamic_blocked_cell_count: {format_value(summary['max_dynamic_blocked_cell_count'])}",
        ]
    )
    if summary.get("active_route_replacement_count") is not None:
        lines.extend(
            [
                f"- active_route_replacement_count: {summary['active_route_replacement_count']}",
                f"- max_active_replan_path_length: {format_value(summary['max_active_replan_path_length'])}",
            ]
        )
    elif summary.get("replan_mode") == "log_only":
        lines.append(
            "- Local replanning was log-only; analyzed replans did not replace active flight waypoints."
        )
    return lines


def markdown_active_replan_route_replacement_summary(summary):
    lines = [
        "## Active Replan Route Replacement Summary",
        "",
    ]
    if summary is None:
        lines.append("- No active replan route replacement columns were found in this log.")
        return lines

    lines.extend(
        [
            f"- active_route_replacement_count: {summary['active_route_replacement_count']}",
            f"- first_active_route_replacement_time_s: {format_value(summary['first_active_route_replacement_time_s'], ' s')}",
            f"- target_before_replacement: {summary['target_before_replacement'] or 'N/A'}",
            f"- first_replanned_target_after_replacement: {summary['first_replanned_target_after_replacement'] or 'N/A'}",
            f"- active_replan_path_length: {format_value(summary['active_replan_path_length'])}",
        ]
    )
    if summary.get("replan_mode") != "active":
        lines.append("- Active route replacement was not enabled for this run.")
    elif summary["active_route_replacement_count"] == 0:
        lines.append("- No active route replacement event was recorded.")
    return lines


def generated_file_description(path):
    descriptions = {
        "summary.md": "human-readable A* analysis report.",
        "manifest.json": "machine-readable analysis metadata.",
        "run_metadata.json": "stage classification and experiment metadata.",
        "traj.png": "clean trajectory overview showing actual path, planned A* waypoints, start, and final/end markers.",
        "traj_with_obstacles.png": "obstacle validation plot showing raw footprints, inflated planning cells, trajectory, and collision/buffer entry points when present.",
        "error.png": "horizontal waypoint tracking error over time.",
        "alt.png": "altitude over time.",
        "vel.png": "NED velocity over time.",
        "yaw.png": "yaw over time.",
        "target_timeline.png": "waypoint target over time.",
        "perception_timeline.png": "nearest simulated obstacle distance over time for active perception samples.",
        "detection_count_over_time.png": "number of simulated obstacle detections at each active perception sample.",
        "perception_risk_timeline.png": "clear/detected/warning/danger risk level over time.",
        "collision_points.csv": "raw physical collision and inflated safety-buffer entry samples.",
        "collision_zoom.png": "zoomed obstacle validation plot around collision/buffer entry samples.",
    }
    return descriptions.get(path.name)


def markdown_generated_file_list(paths):
    if not paths:
        return ["- None"]
    lines = []
    for path in paths:
        description = generated_file_description(path)
        if description:
            lines.append(f"- `{path.name}`: {description}")
        else:
            lines.append(f"- `{path.name}`")
    return lines


def markdown_perception_summary(summary):
    if not summary["available"]:
        return [
            "## Perception Summary",
            "",
            "- perception_enabled: no",
            "- No perception columns were found in this log.",
        ]

    most_frequent = summary["most_frequent_obstacle_names"] or ["N/A"]
    warning_frequent = summary["most_frequent_warning_obstacle_names"] or ["N/A"]
    danger_frequent = summary["most_frequent_danger_obstacle_names"] or ["N/A"]
    risk_note = (
        "- No warning or danger perception samples were recorded."
        if summary["warning_samples"] == 0 and summary["danger_samples"] == 0
        else "- Warning/danger samples were recorded; inspect `perception_risk_timeline.png` and `traj_with_obstacles.png`."
    )
    return [
        "## Perception Summary",
        "",
        f"- perception_enabled: {'yes' if summary['perception_enabled'] else 'no'}",
        f"- total_duration_s: {format_value(summary['total_duration_s'])}",
        f"- detection_range_m: {format_value(summary['detection_range_m'])}",
        f"- detection_fov_deg: {format_value(summary['detection_fov_deg'])}",
        f"- warning_distance_m: {format_value(summary['warning_distance_m'])}",
        f"- danger_distance_m: {format_value(summary['danger_distance_m'])}",
        f"- risk_action: {summary['risk_action'] or 'N/A'}",
        f"- total_detection_samples: {summary['total_detection_samples']}",
        f"- clear_sample_count: {summary['clear_sample_count']}",
        f"- detected_sample_count: {summary['detected_sample_count']}",
        f"- warning_sample_count: {summary['warning_sample_count']}",
        f"- danger_sample_count: {summary['danger_sample_count']}",
        f"- clear_sample_ratio: {format_value(summary['clear_sample_ratio'])}",
        f"- detected_sample_ratio: {format_value(summary['detected_sample_ratio'])}",
        f"- warning_sample_ratio: {format_value(summary['warning_sample_ratio'])}",
        f"- danger_sample_ratio: {format_value(summary['danger_sample_ratio'])}",
        f"- time_in_clear_s: {format_value(summary['time_in_clear_s'], ' s')}",
        f"- time_in_detected_s: {format_value(summary['time_in_detected_s'], ' s')}",
        f"- time_in_warning_s: {format_value(summary['time_in_warning_s'], ' s')}",
        f"- time_in_danger_s: {format_value(summary['time_in_danger_s'], ' s')}",
        f"- percent_time_clear: {format_value(summary['percent_time_clear'], '%')}",
        f"- percent_time_detected: {format_value(summary['percent_time_detected'], '%')}",
        f"- percent_time_warning: {format_value(summary['percent_time_warning'], '%')}",
        f"- percent_time_danger: {format_value(summary['percent_time_danger'], '%')}",
        f"- samples_with_detections: {summary['samples_with_detections']}",
        f"- first detection time: {format_value(summary['first_detection_time_s'], ' s')}",
        f"- first warning time: {format_value(summary['first_warning_time_s'], ' s')}",
        f"- first danger time: {format_value(summary['first_danger_time_s'], ' s')}",
        f"- nearest obstacle ever detected: {summary['nearest_obstacle_ever_detected'] or 'N/A'}",
        f"- minimum_nearest_obstacle_distance_m: {format_value(summary['minimum_nearest_obstacle_distance_m'], ' m')}",
        f"- mean_nearest_obstacle_distance_m: {format_value(summary['mean_nearest_obstacle_distance_m'], ' m')}",
        f"- median_nearest_obstacle_distance_m: {format_value(summary['median_nearest_obstacle_distance_m'], ' m')}",
        f"- most frequently detected obstacle names: {', '.join(most_frequent)}",
        f"- most frequent warning obstacle names: {', '.join(warning_frequent)}",
        f"- most frequent danger obstacle names: {', '.join(danger_frequent)}",
        "",
        "Interpretation:",
        "",
        "- Sample ratios and percent-time metrics are normalized so runs with different durations can be compared more fairly.",
        "- `detected` means an obstacle is within sensor range but not immediately risky.",
        "- `warning` means the drone is close enough to require caution.",
        "- `danger` means the drone is very close and should trigger emergency behavior in future versions.",
        risk_note,
    ]


def write_run_metadata(
    output_dir,
    run_id,
    stage,
    experiment_type,
    log_path,
    obstacle_config_path,
    df,
    infer_local_replan_enabled_func,
    infer_return_home_enabled_func,
):
    metadata = {
        "run_name": run_id,
        "stage": stage,
        "experiment_type": experiment_type,
        "source_log": str(display_path(log_path)),
        "obstacle_config": str(display_path(obstacle_config_path)) if obstacle_config_path else None,
        "perception_enabled": bool(perception_enabled_mask(df).any()) if "perception_enabled" in df.columns else False,
        "risk_action": first_active_value(df, "risk_action") or first_value(df, "risk_action"),
        "local_replan_enabled": infer_local_replan_enabled_func(df),
        "replan_mode": first_value(df, "replan_mode"),
        "altitude_m": safe_max(df, "altitude_m"),
        "max_speed_m_s": safe_max(df, "max_speed_m_s"),
        "return_home": infer_return_home_enabled_func(df),
    }
    metadata_path = output_dir / "run_metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
    return metadata_path


def write_summary(
    output_dir,
    log_path,
    run_id,
    created_at_utc,
    df,
    core_generated_files,
    debug_generated_files,
    collision_generated_files,
    debug_plots_enabled,
    obstacle_config_path,
    collision_report,
    warnings,
    obstacle_map,
    waypoint_transition_summary_func,
    perception_summary_func,
    replan_summary_func,
    active_replan_route_replacement_summary_func,
    infer_return_home_enabled_func,
    waypoint_reached_threshold_m,
):
    summary_path = output_dir / "summary.md"
    planner_name = first_value(df, "planner_name") or "N/A"
    map_name = first_value(df, "map_name") or "N/A"
    transition_rows = waypoint_transition_summary_func(df)
    perception = perception_summary_func(df)
    replan = replan_summary_func(df)
    active_replan_replacement = active_replan_route_replacement_summary_func(df)
    has_obstacle_validation_plot = any(
        path.name == "traj_with_obstacles.png" for path in core_generated_files
    )
    collision_debug_generated = bool(collision_generated_files)

    lines = [
        f"# A* Flight Analysis Summary: {run_id}",
        "",
        f"- Source log: `{display_path(log_path)}`",
        f"- Run ID: `{run_id}`",
        f"- Analysis timestamp: `{created_at_utc}`",
        f"- Map name: `{map_name}`",
        f"- Planner name: `{planner_name}`",
        f"- Row count: {len(df)}",
        f"- Duration: {format_value(duration_s(df), ' s')}",
        f"- Return-home enabled: `{infer_return_home_enabled_func(df)}`",
        f"- Max altitude: {format_value(safe_max(df, 'altitude_m'), ' m')}",
        f"- Max horizontal error: {format_value(safe_max(df, 'horizontal_error_m'), ' m')}",
        f"- Mean horizontal error: {format_value(safe_mean(df, 'horizontal_error_m'), ' m')}",
        f"- Final horizontal error: {format_value(safe_last_valid(df, 'horizontal_error_m'), ' m')}",
        f"- Waypoint reached threshold: {waypoint_reached_threshold_m:.3f} m",
        f"- Obstacle config: `{display_path(obstacle_config_path) if obstacle_config_path else 'N/A'}`",
        f"- Height-aware altitude: {format_value(obstacle_map.get('altitude_m') if obstacle_map else None, ' m')}",
        f"- Vertical safety margin: {format_value(obstacle_map.get('vertical_safety_margin_m') if obstacle_map else None, ' m')}",
        f"- Horizontal inflation cells: {obstacle_map.get('horizontal_inflation_cells') if obstacle_map else 'N/A'}",
        f"- Blocking obstacles: `{obstacle_map.get('blocking_obstacle_names') if obstacle_map else []}`",
        f"- Ignored/nonblocking obstacles: `{obstacle_map.get('nonblocking_obstacle_names') if obstacle_map else []}`",
        f"- Raw physical footprint cells: {obstacle_map.get('raw_obstacle_cell_count') if obstacle_map else 'N/A'}",
        f"- Height-blocking raw footprint cells: {obstacle_map.get('raw_blocking_cell_count') if obstacle_map else 'N/A'}",
        f"- Inflated blocking cells: {obstacle_map.get('inflated_obstacle_cell_count') if obstacle_map else 'N/A'}",
        f"- raw_physical_collision_detected: {'yes' if collision_report['raw_physical_collision_detected'] else 'no'}",
        f"- inflated_safety_buffer_entry_detected: {'yes' if collision_report['inflated_safety_buffer_entry_detected'] else 'no'}",
        f"- Raw physical footprint entry points: {collision_report['raw_collision_points']}",
        f"- Inflated buffer entry points: {collision_report['inflated_buffer_entry_points']}",
        f"- First raw collision timestamps: {collision_report['first_raw_collision_timestamps']}",
        f"- First inflated buffer entry timestamps: {collision_report['first_inflated_buffer_entry_timestamps']}",
        f"- Raw obstacle names involved: `{collision_report['raw_obstacle_names_involved']}`",
        f"- Inflated obstacle names involved: `{collision_report['inflated_obstacle_names_involved']}`",
        f"- approximate_min_clearance_m: {format_value(collision_report['approximate_min_clearance_m'], ' m')}",
        "",
        "## Collision Interpretation",
        "",
        "- Raw physical footprint entries mean logged local XY entered cells corresponding to Gazebo object footprints.",
        "- Inflated safety-buffer entries mean logged local XY entered cells A* treats as blocked after inflation; this may be a clearance violation rather than a physical collision.",
    ]
    if has_obstacle_validation_plot:
        lines.append(
            "- Coarse grid rectangles can create false positives around thin poles; inspect `traj_with_obstacles.png` and the real SDF geometry."
        )
    else:
        lines.append(
            "- No obstacle validation plot was generated because no obstacle overlay was available."
        )
    if collision_debug_generated:
        lines.append("- Collision debug files were generated because raw physical collision or inflated buffer entries were detected.")
    else:
        lines.append("- No collision/buffer entries detected; collision debug files were not generated.")

    lines.extend([
        "",
        "## Waypoint Transition Summary",
        "",
        *markdown_transition_table(transition_rows),
        "",
        *markdown_replan_summary(replan),
        "",
        *markdown_active_replan_route_replacement_summary(active_replan_replacement),
        "",
        *markdown_perception_summary(perception),
        "",
        "## Warnings",
        "",
    ])

    if warnings:
        for warning in warnings:
            lines.append(f"- WARNING: {warning}")
    else:
        lines.append("- None")

    lines.extend(["", "## Generated Files", ""])
    lines.extend(["### Core Files", ""])
    lines.extend(markdown_generated_file_list(core_generated_files))
    lines.extend(["", "### Debug Files", ""])
    if debug_plots_enabled:
        lines.extend(markdown_generated_file_list(debug_generated_files))
    else:
        lines.append(
            "- Debug plots were not generated. Re-run with --debug-plots to create altitude, velocity, yaw, waypoint target, and perception count plots."
        )
    lines.extend(["", "### Collision Debug Files", ""])
    lines.extend(markdown_generated_file_list(collision_generated_files))

    summary_path.write_text("\n".join(lines) + "\n")
    return summary_path


def write_manifest(
    output_dir,
    log_path,
    run_id,
    created_at_utc,
    df,
    core_generated_files,
    debug_generated_files,
    collision_generated_files,
    debug_plots_enabled,
    obstacle_config_path,
    warnings,
    obstacle_map,
    collision_report,
    perception_summary_func,
    replan_summary_func,
    active_replan_route_replacement_summary_func,
):
    manifest_path = output_dir / "manifest.json"
    perception = perception_summary_func(df)
    replan = replan_summary_func(df)
    active_replan_replacement = active_replan_route_replacement_summary_func(df)
    manifest = {
        "run_id": run_id,
        "source_log": str(display_path(log_path)),
        "output_dir": str(display_path(output_dir)),
        "created_at_utc": created_at_utc,
        "row_count": len(df),
        "duration_s": duration_s(df),
        "debug_plots_enabled": debug_plots_enabled,
        "planner_name": first_value(df, "planner_name"),
        "map_name": first_value(df, "map_name"),
        "obstacle_config": str(display_path(obstacle_config_path)) if obstacle_config_path else None,
        "height_aware_planning": {
            "altitude_m": obstacle_map.get("altitude_m") if obstacle_map else None,
            "vertical_safety_margin_m": (
                obstacle_map.get("vertical_safety_margin_m") if obstacle_map else None
            ),
            "horizontal_inflation_cells": (
                obstacle_map.get("horizontal_inflation_cells") if obstacle_map else None
            ),
            "blocking_obstacle_names": (
                obstacle_map.get("blocking_obstacle_names") if obstacle_map else []
            ),
            "nonblocking_obstacle_names": (
                obstacle_map.get("nonblocking_obstacle_names") if obstacle_map else []
            ),
            "obstacle_cell_count": obstacle_map.get("obstacle_cell_count") if obstacle_map else None,
            "raw_obstacle_cell_count": (
                obstacle_map.get("raw_obstacle_cell_count") if obstacle_map else None
            ),
            "raw_blocking_cell_count": (
                obstacle_map.get("raw_blocking_cell_count") if obstacle_map else None
            ),
            "inflated_obstacle_cell_count": (
                obstacle_map.get("inflated_obstacle_cell_count") if obstacle_map else None
            ),
        },
        "collision_report": {
            "raw_physical_collision_detected": collision_report["raw_physical_collision_detected"],
            "inflated_safety_buffer_entry_detected": collision_report[
                "inflated_safety_buffer_entry_detected"
            ],
            "raw_collision_points": collision_report["raw_collision_points"],
            "inflated_buffer_entry_points": collision_report["inflated_buffer_entry_points"],
            "first_raw_collision_timestamps": collision_report["first_raw_collision_timestamps"],
            "first_inflated_buffer_entry_timestamps": collision_report[
                "first_inflated_buffer_entry_timestamps"
            ],
            "raw_obstacle_names_involved": collision_report["raw_obstacle_names_involved"],
            "inflated_obstacle_names_involved": collision_report[
                "inflated_obstacle_names_involved"
            ],
        },
        "replan_summary": replan,
        "active_replan_route_replacement_summary": active_replan_replacement,
        "perception_summary": perception,
        "core_generated_files": [str(display_path(path)) for path in core_generated_files],
        "debug_generated_files": [str(display_path(path)) for path in debug_generated_files],
        "collision_generated_files": [
            str(display_path(path)) for path in collision_generated_files
        ],
        "generated_files": [
            str(display_path(path))
            for path in [
                *core_generated_files,
                *debug_generated_files,
                *collision_generated_files,
            ]
        ],
        "warnings": warnings,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest_path
