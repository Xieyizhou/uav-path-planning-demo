"""Plotting helpers for offline A* analysis reports."""

import math

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from src.logging.log_io import display_path
from src.logging.metrics import (
    DEFAULT_DANGER_DISTANCE_M,
    DEFAULT_WARNING_DISTANCE_M,
    RISK_LEVEL_TO_VALUE,
    detected_obstacle_mask,
    first_active_value,
    has_perception_columns,
    perception_enabled_mask,
    risk_level_series,
)


WAYPOINT_REACHED_THRESHOLD_M = 0.4
PLOT_SPLIT_DISTANCE_M = 3.0


def save_line_plot(df, columns, title, ylabel, output_path, source_log=None):
    available_columns = [column for column in columns if column in df.columns]
    if not available_columns:
        print(f"Skipping {title}: missing columns {columns}")
        return None

    plot_df = df[["elapsed_s", *available_columns]].dropna(
        subset=available_columns,
        how="all",
    )
    if plot_df.empty:
        print(f"Skipping {title}: no non-empty data points")
        return None

    plt.figure(figsize=(10, 5))
    for column in available_columns:
        plt.plot(plot_df["elapsed_s"], plot_df[column], label=column)
    subtitle = f"\nSource: {source_log}" if source_log else ""
    plt.title(f"{title}{subtitle}")
    plt.xlabel("Elapsed time (s)")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved plot: {output_path}")
    return output_path


def save_perception_timeline(df, output_path, source_log):
    required_columns = {"elapsed_s", "perception_enabled", "nearest_obstacle_distance_m"}
    if not required_columns.issubset(df.columns):
        print("Skipping perception timeline: perception distance columns are missing")
        return None

    plot_df = df[perception_enabled_mask(df)].dropna(
        subset=["elapsed_s", "nearest_obstacle_distance_m"]
    )
    if plot_df.empty:
        print("Skipping perception timeline: no active detection distance samples")
        return None

    plt.figure(figsize=(10, 5))
    plt.plot(
        plot_df["elapsed_s"],
        plot_df["nearest_obstacle_distance_m"],
        marker="o",
        markersize=3,
        linewidth=1.2,
        label="nearest detected obstacle",
    )
    warning_distance = first_active_value(df, "warning_distance_m")
    danger_distance = first_active_value(df, "danger_distance_m")
    warning_distance = DEFAULT_WARNING_DISTANCE_M if warning_distance is None else warning_distance
    danger_distance = DEFAULT_DANGER_DISTANCE_M if danger_distance is None else danger_distance
    if warning_distance is not None:
        plt.axhline(
            float(warning_distance),
            color="orange",
            linestyle="--",
            linewidth=1.1,
            label="warning threshold",
        )
    if danger_distance is not None:
        plt.axhline(
            float(danger_distance),
            color="red",
            linestyle="--",
            linewidth=1.1,
            label="danger threshold",
        )
    plt.title(f"Perception Nearest Obstacle Distance\nSource: {source_log}")
    plt.xlabel("Elapsed time (s)")
    plt.ylabel("Nearest obstacle distance (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved plot: {output_path}")
    return output_path


def save_perception_risk_timeline(df, output_path, source_log):
    required_columns = {"elapsed_s", "perception_enabled"}
    if not required_columns.issubset(df.columns):
        print("Skipping perception risk timeline: perception columns are missing")
        return None

    plot_df = df[perception_enabled_mask(df)].copy()
    if plot_df.empty:
        print("Skipping perception risk timeline: no active perception samples")
        return None

    plot_df["risk_value"] = risk_level_series(plot_df).map(RISK_LEVEL_TO_VALUE)
    plot_df = plot_df.dropna(subset=["elapsed_s", "risk_value"])
    if plot_df.empty:
        print("Skipping perception risk timeline: no risk-level samples")
        return None

    risk_action = first_active_value(df, "risk_action") or "log_only"
    plt.figure(figsize=(10, 5))
    plt.step(
        plot_df["elapsed_s"],
        plot_df["risk_value"],
        where="post",
        linewidth=1.4,
        label="perception risk level",
    )
    plt.yticks(
        [0, 1, 2, 3],
        ["clear = 0", "detected = 1", "warning = 2", "danger = 3"],
    )
    plt.ylim(-0.25, 3.25)
    plt.title(
        "Perception Risk Level Over Time"
        f"\nSource: {source_log} | risk_action: {risk_action}"
    )
    plt.xlabel("Elapsed time (s)")
    plt.ylabel("Risk level (clear=0, detected=1, warning=2, danger=3)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved plot: {output_path}")
    return output_path


def save_detection_count_plot(df, output_path, source_log):
    required_columns = {"elapsed_s", "perception_enabled", "detected_obstacle_count"}
    if not required_columns.issubset(df.columns):
        print("Skipping detection count plot: perception count columns are missing")
        return None

    plot_df = df[perception_enabled_mask(df)].dropna(
        subset=["elapsed_s", "detected_obstacle_count"]
    )
    if plot_df.empty:
        print("Skipping detection count plot: no active perception count samples")
        return None

    plt.figure(figsize=(10, 5))
    plt.plot(
        plot_df["elapsed_s"],
        plot_df["detected_obstacle_count"],
        marker=".",
        linewidth=1.1,
        label="detected obstacle count",
    )
    plt.title(f"Perception Detection Count Over Time\nSource: {source_log}")
    plt.xlabel("Elapsed time (s)")
    plt.ylabel("Detected obstacle count")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved plot: {output_path}")
    return output_path


def save_error_plot(df, output_path, source_log):
    if "horizontal_error_m" not in df.columns:
        print("Skipping error plot: horizontal_error_m is missing")
        return None

    plot_df = df[["elapsed_s", "horizontal_error_m", "target_name"]].dropna(
        subset=["elapsed_s", "horizontal_error_m"]
    )
    if plot_df.empty:
        print("Skipping error plot: no horizontal error data")
        return None

    plt.figure(figsize=(10, 5))
    plt.plot(plot_df["elapsed_s"], plot_df["horizontal_error_m"], label="horizontal_error_m")
    plt.axhline(
        WAYPOINT_REACHED_THRESHOLD_M,
        color="tab:green",
        linestyle="--",
        linewidth=1,
        label=f"reached threshold {WAYPOINT_REACHED_THRESHOLD_M:.1f} m",
    )

    if "target_name" in plot_df.columns:
        last_target = None
        for _, row in plot_df.iterrows():
            target = str(row["target_name"])
            if target and target != last_target:
                plt.axvline(row["elapsed_s"], color="0.75", linestyle=":", linewidth=0.8)
                last_target = target

    plt.title(f"Horizontal A* Waypoint Error Over Time\nSource: {source_log}")
    plt.xlabel("Elapsed time (s)")
    plt.ylabel("Horizontal error (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved plot: {output_path}")
    return output_path


def target_sequence(df, route_direction=None):
    required_columns = ["target_name", "target_north_m", "target_east_m", "target_down_m"]
    if not all(column in df.columns for column in required_columns):
        return []

    target_df = df[required_columns + [col for col in ["route_direction"] if col in df.columns]].dropna(
        subset=["target_north_m", "target_east_m"]
    )
    if route_direction is not None and "route_direction" in target_df.columns:
        target_df = target_df[target_df["route_direction"] == route_direction]

    targets = []
    seen = set()
    for _, row in target_df.iterrows():
        name = str(row["target_name"])
        if not name or name == "nan":
            continue
        key = (route_direction, name)
        if key in seen:
            continue
        targets.append(
            {
                "name": name,
                "north_m": float(row["target_north_m"]),
                "east_m": float(row["target_east_m"]),
                "down_m": float(row["target_down_m"]),
            }
        )
        seen.add(key)
    return targets


def plot_obstacle_cells(ax, obstacles, resolution_m, label, facecolor, alpha, hatch=None):
    if not obstacles:
        return
    first = True
    for x, y in obstacles:
        ax.add_patch(
            Rectangle(
                (x * resolution_m, y * resolution_m),
                resolution_m,
                resolution_m,
                facecolor=facecolor,
                edgecolor="0.25",
                alpha=alpha,
                hatch=hatch,
                label=label if first else None,
            )
        )
        first = False


def plot_obstacle_layers(ax, obstacle_map, resolution_m):
    if not obstacle_map or not resolution_m:
        return
    plot_obstacle_cells(
        ax,
        obstacle_map.get("raw_obstacle_cells", set()),
        resolution_m,
        "raw physical footprint",
        "0.78",
        0.60,
    )
    plot_obstacle_cells(
        ax,
        obstacle_map.get("inflated_blocking_cells", set()),
        resolution_m,
        "inflated planning obstacle",
        "0.25",
        0.25,
        hatch="//",
    )


def trajectory_segments(df):
    required_columns = ["elapsed_s", "local_north_m", "local_east_m"]
    if not all(column in df.columns for column in required_columns):
        return []

    columns = required_columns + [
        column
        for column in ["phase", "route_direction", "target_name"]
        if column in df.columns
    ]
    plot_df = df[columns].dropna(subset=required_columns).sort_values("elapsed_s")
    if plot_df.empty:
        return []

    segments = []
    current_rows = []
    previous = None
    for _, row in plot_df.iterrows():
        split = False
        if previous is not None:
            dx = float(row["local_east_m"]) - float(previous["local_east_m"])
            dy = float(row["local_north_m"]) - float(previous["local_north_m"])
            jump = math.hypot(dx, dy)
            if jump > PLOT_SPLIT_DISTANCE_M:
                split = True
            if "phase" in plot_df.columns and row["phase"] != previous["phase"]:
                split = True
            if "route_direction" in plot_df.columns and row["route_direction"] != previous["route_direction"]:
                split = True
            if (
                "target_name" in plot_df.columns
                and row["target_name"] != previous["target_name"]
                and jump > PLOT_SPLIT_DISTANCE_M
            ):
                split = True

        if split and current_rows:
            segments.append(pd.DataFrame(current_rows))
            current_rows = []
        current_rows.append(row.to_dict())
        previous = row

    if current_rows:
        segments.append(pd.DataFrame(current_rows))
    return segments


def annotate_waypoints(ax, targets):
    offsets = [(6, 8), (8, -12), (-22, 8), (-24, -12), (10, 16), (-32, 0)]
    for index, target in enumerate(targets):
        offset = offsets[index % len(offsets)]
        ax.annotate(
            target["name"],
            (target["east_m"], target["north_m"]),
            textcoords="offset points",
            xytext=offset,
            fontsize=8,
            bbox={"boxstyle": "round,pad=0.15", "fc": "white", "alpha": 0.7, "lw": 0},
        )


def save_trajectory_plot(
    df,
    output_path,
    log_path,
    map_name,
    obstacle_map,
    resolution_m,
    collision_report=None,
    title="A* Local 2D Trajectory",
    show_obstacles=False,
    show_collision_points=False,
    show_perception_points=False,
):
    required_columns = ["local_north_m", "local_east_m"]
    if not all(column in df.columns for column in required_columns):
        print("Skipping trajectory plot: local position columns are missing")
        return None

    valid_df = df[required_columns].dropna()
    if valid_df.empty:
        print("Skipping trajectory plot: no valid local position data")
        return None

    fig, ax = plt.subplots(figsize=(8, 8))
    if show_obstacles:
        plot_obstacle_layers(ax, obstacle_map, resolution_m)

    colors = {
        "outbound": "tab:blue",
        "return": "tab:purple",
        "none": "0.45",
        "": "0.45",
    }
    labels_seen = set()
    for segment in trajectory_segments(df):
        route_direction = str(segment["route_direction"].iloc[0]) if "route_direction" in segment.columns else "none"
        phase = str(segment["phase"].iloc[0]) if "phase" in segment.columns else ""
        if route_direction == "outbound":
            label = "actual outbound path"
        elif route_direction == "return":
            label = "actual return path"
        elif phase in {"goal_hover", "start_hover", "landing", "landed"}:
            label = f"actual {phase}"
        else:
            label = "actual other/hover"
        ax.plot(
            segment["local_east_m"],
            segment["local_north_m"],
            marker="o",
            markersize=2,
            linewidth=1.4 if route_direction in {"outbound", "return"} else 0.9,
            color=colors.get(route_direction, "0.45"),
            alpha=0.9 if route_direction in {"outbound", "return"} else 0.55,
            label=label if label not in labels_seen else None,
        )
        labels_seen.add(label)

    outbound_targets = target_sequence(df, route_direction="outbound")
    if not outbound_targets:
        outbound_targets = target_sequence(df)
    if outbound_targets:
        target_east = [target["east_m"] for target in outbound_targets]
        target_north = [target["north_m"] for target in outbound_targets]
        ax.plot(
            target_east,
            target_north,
            "x--",
            color="tab:orange",
            markersize=8,
            linewidth=1.5,
            label="planned A* waypoints",
        )
        annotate_waypoints(ax, outbound_targets)

    if show_collision_points and collision_report:
        raw_rows = collision_report.get("raw_collision_rows", [])
        inflated_rows = collision_report.get("inflated_buffer_entry_rows", [])
        if raw_rows:
            raw_df = pd.DataFrame(raw_rows)
            ax.scatter(
                raw_df["local_east_m"],
                raw_df["local_north_m"],
                color="red",
                s=30,
                marker="x",
                label="raw footprint entry",
                zorder=5,
            )
        if inflated_rows:
            inflated_df = pd.DataFrame(inflated_rows)
            ax.scatter(
                inflated_df["local_east_m"],
                inflated_df["local_north_m"],
                facecolors="none",
                edgecolors="black",
                s=36,
                marker="o",
                label="inflated buffer entry",
                zorder=5,
            )

    if show_perception_points and has_perception_columns(df):
        perception_df = df[
            detected_obstacle_mask(df)
            & df["local_east_m"].notna()
            & df["local_north_m"].notna()
        ].copy()
        if not perception_df.empty:
            perception_df["risk_level"] = risk_level_series(perception_df)
        detected_df = perception_df[perception_df["risk_level"] == "detected"] if not perception_df.empty else pd.DataFrame()
        warning_df = perception_df[perception_df["risk_level"] == "warning"] if not perception_df.empty else pd.DataFrame()
        danger_df = perception_df[perception_df["risk_level"] == "danger"] if not perception_df.empty else pd.DataFrame()
        if not detected_df.empty:
            ax.scatter(
                detected_df["local_east_m"],
                detected_df["local_north_m"],
                color="tab:blue",
                s=10,
                marker="o",
                alpha=0.25,
                label="perception detected sample",
                zorder=5,
            )
        if not warning_df.empty:
            ax.scatter(
                warning_df["local_east_m"],
                warning_df["local_north_m"],
                color="orange",
                s=28,
                marker="o",
                alpha=0.8,
                label="perception warning sample",
                zorder=6,
            )
        if not danger_df.empty:
            ax.scatter(
                danger_df["local_east_m"],
                danger_df["local_north_m"],
                color="red",
                s=34,
                marker="o",
                alpha=0.9,
                label="perception danger sample",
                zorder=6,
            )

    first_position = df[["local_east_m", "local_north_m"]].dropna().iloc[0]
    final_position = df[["local_east_m", "local_north_m"]].dropna().iloc[-1]
    ax.scatter(
        first_position["local_east_m"],
        first_position["local_north_m"],
        marker="o",
        s=120,
        color="limegreen",
        edgecolor="black",
        label="start",
        zorder=4,
    )
    ax.scatter(
        final_position["local_east_m"],
        final_position["local_north_m"],
        marker="*",
        s=180,
        color="red",
        edgecolor="black",
        label="final/end",
        zorder=4,
    )

    map_text = map_name or "N/A"
    ax.set_title(
        f"{title}\n"
        f"Source: {display_path(log_path)} | Map: {map_text}"
    )
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot: {output_path}")
    return output_path


def save_target_timeline(df, output_path, source_log):
    required_columns = ["elapsed_s", "target_name"]
    if not all(column in df.columns for column in required_columns):
        print("Skipping target timeline: target_name or elapsed_s is missing")
        return None

    columns = ["elapsed_s", "target_name"]
    if "route_direction" in df.columns:
        columns.append("route_direction")
    plot_df = df[columns].dropna(subset=["elapsed_s"])
    if "route_direction" not in plot_df.columns:
        plot_df["route_direction"] = "none"
    plot_df = plot_df[plot_df["target_name"].astype(str) != ""]
    if plot_df.empty:
        print("Skipping target timeline: no target data")
        return None

    ordered_targets = []
    for name in plot_df["target_name"]:
        if name not in ordered_targets:
            ordered_targets.append(name)
    target_to_index = {target: index for index, target in enumerate(ordered_targets, start=1)}
    plot_df = plot_df.copy()
    plot_df["target_index"] = plot_df["target_name"].map(target_to_index)

    plt.figure(figsize=(10, 5))
    for route_direction, group in plot_df.groupby("route_direction", dropna=False):
        label = route_direction if route_direction else "none"
        plt.scatter(group["elapsed_s"], group["target_index"], s=12, label=label)
    plt.yticks(list(target_to_index.values()), list(target_to_index.keys()))
    plt.title(f"A* Target Timeline\nSource: {source_log}")
    plt.xlabel("Elapsed time (s)")
    plt.ylabel("Target waypoint")
    plt.grid(True, axis="x")
    plt.legend(title="route_direction")
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved plot: {output_path}")
    return output_path


def save_collision_zoom_plot(df, output_path, obstacle_map, resolution_m, collision_report):
    rows = collision_report.get("collision_rows", [])
    position_df = df[["local_east_m", "local_north_m"]].dropna()
    if rows:
        collision_df = pd.DataFrame(rows)
        east_min = collision_df["local_east_m"].min() - 3
        east_max = collision_df["local_east_m"].max() + 3
        north_min = collision_df["local_north_m"].min() - 3
        north_max = collision_df["local_north_m"].max() + 3
    elif not position_df.empty:
        east_min = position_df["local_east_m"].min() - 1
        east_max = position_df["local_east_m"].max() + 1
        north_min = position_df["local_north_m"].min() - 1
        north_max = position_df["local_north_m"].max() + 1
    else:
        return None

    fig, ax = plt.subplots(figsize=(7, 7))
    plot_obstacle_layers(ax, obstacle_map, resolution_m)
    ax.plot(
        position_df["local_east_m"],
        position_df["local_north_m"],
        color="tab:blue",
        linewidth=1.2,
        label="actual trajectory",
    )
    if rows:
        raw_df = collision_df[collision_df["entry_type"] == "raw_physical"]
        inflated_df = collision_df[collision_df["entry_type"] == "inflated_buffer"]
        if not raw_df.empty:
            ax.scatter(
                raw_df["local_east_m"],
                raw_df["local_north_m"],
                color="red",
                s=32,
                marker="x",
                label="raw footprint entry",
                zorder=4,
            )
        if not inflated_df.empty:
            ax.scatter(
                inflated_df["local_east_m"],
                inflated_df["local_north_m"],
                facecolors="none",
                edgecolors="black",
                s=36,
                marker="o",
                label="inflated buffer entry",
                zorder=4,
            )
    else:
        ax.text(
            0.5,
            0.95,
            "No hard obstacle cell entries detected",
            transform=ax.transAxes,
            ha="center",
            va="top",
            bbox={"boxstyle": "round,pad=0.25", "fc": "white", "alpha": 0.8},
        )
    ax.set_xlim(east_min, east_max)
    ax.set_ylim(north_min, north_max)
    ax.set_title("Obstacle Collision Zoom")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot: {output_path}")
    return output_path
