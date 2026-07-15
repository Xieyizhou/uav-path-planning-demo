import argparse
import csv
import json
import statistics
import sys
from datetime import datetime, timezone
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.logging.output_registry import (
    RUN_STAGES,
    STAGE_DIR_NAMES,
    STAGE_LABELS,
    display_path,
    ensure_output_tree,
    get_comparisons_dir,
    get_runs_dir,
)
from src.logging.evaluation_schema import EVALUATION_COLUMNS
from src.logging.summarize_experiments import (
    collect_run,
    first_non_empty,
    parse_float,
    read_text,
    normalized_evaluation_row,
    to_evaluation_row,
)


MARKERS = [
    "KEEP_BASELINE__astar_only_perception_disabled.txt",
    "KEEP_COMPARISON__perception_slow_down.txt",
    "KEEP_CONTROL__log_only_local_replan_no_route_replacement.txt",
    "KEEP_LANDMARK__active_local_replan.txt",
]

LANDMARK_COMPARISON_NAME = "landmark"
AGGREGATE_COMPARISON_NAME = "aggregate"
COMPARISON_COLUMNS = [*EVALUATION_COLUMNS, "marker", "run_dir"]
REQUIRED_STAGES = RUN_STAGES
STATUS_FILENAME = "comparison_status.md"
AGGREGATE_COLUMNS = [
    "stage",
    "stage_folder",
    "run_count",
    "completed_count",
    "pass_count",
    "mean_total_flight_time_s",
    "std_total_flight_time_s",
    "min_total_flight_time_s",
    "max_total_flight_time_s",
    "mean_planned_path_length_m",
    "mean_actual_traveled_distance_m",
    "mean_minimum_distance_to_obstacle_m",
    "total_safety_buffer_violation_count",
    "mean_perception_risk_detection_count",
    "mean_slow_down_event_count",
    "mean_local_replan_attempt_count",
    "mean_successful_local_replan_count",
    "mean_active_route_replacement_count",
]
INCLUDED_RUN_COLUMNS = [*EVALUATION_COLUMNS, "run_dir"]


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Generate landmark and aggregate cross-stage comparisons for the "
            "four official experiment stages."
        )
    )
    parser.add_argument(
        "--mode",
        choices=["landmark", "aggregate", "both"],
        default="both",
        help="Comparison output mode. Default: both",
    )
    parser.add_argument(
        "--min-runs-per-stage",
        type=int,
        default=1,
        help="Minimum valid analyzed runs required per official stage. Default: 1",
    )
    parser.add_argument(
        "--allow-partial",
        action="store_true",
        help="Write comparison outputs even when required stages are missing.",
    )
    parser.add_argument(
        "--strategy",
        choices=["latest-complete"],
        default="latest-complete",
        help="Selection strategy for landmark runs. Default: latest-complete",
    )
    return parser.parse_args()


def load_json(path):
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError as error:
        print(f"WARNING: Could not parse {display_path(path)}: {error}")
        return {}


def marker_search_roots():
    return [get_runs_dir(stage_name) for stage_name in RUN_STAGES]


def find_marker(marker_name):
    candidates = []
    for root in marker_search_roots():
        if not root.exists():
            continue
        candidates.extend(root.glob(f"**/{marker_name}"))
    candidates = sorted(
        {path for path in candidates if path.parent.name.startswith("as_")},
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    return candidates[0] if candidates else None


def infer_stage_from_path(run_dir, metadata):
    if metadata.get("stage"):
        return metadata["stage"]
    parts = set(run_dir.parts)
    if "01_static_astar" in parts:
        return "static_astar"
    if "02_perception_response" in parts:
        return "perception_response"
    if "03_replan_log_only" in parts:
        return "replan_log_only"
    if "04_active_replan" in parts:
        return "active_replan"
    return "legacy_unmigrated"


def infer_experiment_type(marker_name, metadata):
    if metadata.get("experiment_type"):
        return metadata["experiment_type"]
    mapping = {
        "KEEP_BASELINE__astar_only_perception_disabled.txt": "astar_only_perception_disabled",
        "KEEP_COMPARISON__perception_slow_down.txt": "perception_slow_down",
        "KEEP_CONTROL__log_only_local_replan_no_route_replacement.txt": "local_replan_log_only",
        "KEEP_LANDMARK__active_local_replan.txt": "active_local_replan",
    }
    return mapping.get(marker_name)


def selected_run(marker_name, marker_path):
    run_dir = marker_path.parent
    manifest = load_json(run_dir / "manifest.json")
    metadata = load_json(run_dir / "run_metadata.json")
    summary_text = read_text(run_dir / "summary.md")
    collected = collect_run(run_dir)
    evaluation = to_evaluation_row(collected)
    evaluation.update(
        {
            "stage": infer_stage_from_path(run_dir, metadata),
            "stage_folder": STAGE_DIR_NAMES.get(infer_stage_from_path(run_dir, metadata)),
            "experiment_type": infer_experiment_type(marker_name, metadata),
            "marker": marker_name,
            "run_dir": display_path(run_dir),
            "summary_available": bool(summary_text),
            "manifest_available": bool(manifest),
            "run_metadata_available": bool(metadata),
        }
    )
    if not evaluation.get("run_id"):
        evaluation["run_id"] = first_non_empty(metadata.get("run_name"), collected.get("run_id"), run_dir.name)
    return {
        "marker": marker_name,
        "run_dir": display_path(run_dir),
        "summary_available": bool(summary_text),
        "manifest_available": bool(manifest),
        "run_metadata_available": bool(metadata),
        **evaluation,
    }


def find_selected_runs():
    selected = []
    for marker_name in MARKERS:
        marker_path = find_marker(marker_name)
        if marker_path is None:
            print(f"WARNING: marker not found: {marker_name}")
            continue
        selected.append(selected_run(marker_name, marker_path))
    return selected


def analyzed_run(stage_name, run_dir):
    manifest = load_json(run_dir / "manifest.json")
    metadata = load_json(run_dir / "run_metadata.json")
    summary_text = read_text(run_dir / "summary.md")
    collected = collect_run(run_dir)
    evaluation = to_evaluation_row(collected)
    inferred_stage = evaluation.get("stage") or metadata.get("stage") or stage_name
    evaluation.update(
        {
            "stage": inferred_stage,
            "stage_folder": STAGE_DIR_NAMES.get(inferred_stage),
            "run_dir": display_path(run_dir),
            "summary_available": bool(summary_text),
            "manifest_available": bool(manifest),
            "run_metadata_available": bool(metadata),
        }
    )
    if not evaluation.get("run_id"):
        evaluation["run_id"] = first_non_empty(metadata.get("run_name"), collected.get("run_id"), run_dir.name)
    return evaluation


def find_all_valid_runs():
    rows = []
    for stage_name in RUN_STAGES:
        runs_dir = get_runs_dir(stage_name)
        if not runs_dir.exists():
            continue
        for run_dir in sorted(runs_dir.glob("as_*")):
            if not run_dir.is_dir():
                continue
            if not (run_dir / "manifest.json").exists() and not (run_dir / "summary.md").exists():
                continue
            row = analyzed_run(stage_name, run_dir)
            if is_analyzed_run(row):
                rows.append(row)
    return rows


def is_analyzed_run(row):
    return bool(row.get("run_id")) and bool(
        row.get("summary_available") or row.get("manifest_available")
    )


def valid_stage_counts(rows):
    counts = {stage_name: 0 for stage_name in REQUIRED_STAGES}
    for row in rows:
        stage_name = row.get("stage")
        if stage_name not in counts:
            continue
        if is_analyzed_run(row):
            counts[stage_name] += 1
    return counts


def missing_required_stages(rows, min_runs_per_stage):
    counts = valid_stage_counts(rows)
    return [
        stage_name
        for stage_name in REQUIRED_STAGES
        if counts.get(stage_name, 0) < min_runs_per_stage
    ]


def print_stage_counts(rows):
    counts = valid_stage_counts(rows)
    for stage_name in REQUIRED_STAGES:
        print(f"{stage_name}: {counts.get(stage_name, 0)} valid run(s)")


def write_status(output_path, rows, missing_stages, min_runs_per_stage, generated):
    created_at = datetime.now(timezone.utc).isoformat()
    counts = valid_stage_counts(rows)
    lines = [
        "# Cross-Stage Comparison Status",
        "",
        f"Generated: `{created_at}`",
        "",
        f"Minimum analyzed runs per stage: `{min_runs_per_stage}`",
        "",
        "| stage | analyzed selected runs | status |",
        "|---|---:|---|",
    ]
    for stage_name in REQUIRED_STAGES:
        count = counts.get(stage_name, 0)
        status = "ok" if stage_name not in missing_stages else "missing"
        lines.append(f"| `{STAGE_DIR_NAMES[stage_name]}` | {count} | {status} |")

    lines.extend(["", "## Result", ""])
    if generated:
        lines.append("Comparison outputs were generated.")
    else:
        lines.append(
            "Comparison outputs were skipped because at least one required stage "
            "does not have enough selected analyzed runs."
        )
        lines.extend(["", "Missing stages:", ""])
        for stage_name in missing_stages:
            lines.append(f"- `{STAGE_DIR_NAMES[stage_name]}`")
        lines.extend(
            [
                "",
                "Existing `comparison_summary.csv` and `comparison_summary.md` files were left unchanged.",
            ]
        )
    output_path.write_text("\n".join(lines) + "\n")


def normalized_value(column, value):
    if column in EVALUATION_COLUMNS:
        return normalized_evaluation_row({column: value}).get(column, "")
    return "" if value is None else str(value)


def write_csv(rows, output_path):
    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=COMPARISON_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {column: normalized_value(column, row.get(column)) for column in COMPARISON_COLUMNS}
            )


def md_value(row, column):
    value = normalized_value(column, row.get(column))
    return value if value != "" else "n/a"


def interpretation(rows):
    if not rows:
        return ["- No marker-selected landmark runs were found."]
    lines = [
        "- This is an intentional cross-stage comparison; ordinary stage summaries remain separate.",
    ]
    baseline = next((row for row in rows if row.get("experiment_type") == "astar_only_perception_disabled"), None)
    slow_down = next((row for row in rows if row.get("experiment_type") == "perception_slow_down"), None)
    active = next((row for row in rows if row.get("experiment_type") == "active_local_replan"), None)
    if baseline and slow_down:
        b_clearance = parse_float(baseline.get("minimum_distance_to_obstacle_m"))
        s_clearance = parse_float(slow_down.get("minimum_distance_to_obstacle_m"))
        if b_clearance is not None and s_clearance is not None:
            relation = "higher" if s_clearance > b_clearance else "not higher"
            lines.append(f"- Slow-down minimum obstacle distance is {relation} than the static baseline.")
    if active:
        replacements = parse_float(active.get("active_route_replacement_count"))
        if replacements and replacements > 0:
            lines.append("- The active replan landmark recorded route replacement, so it changed the outbound route rather than only logging a possible replan.")
        else:
            lines.append("- The active replan landmark did not record a route replacement in the available metrics.")
    if any(row.get("completed_or_failed") == "failed" for row in rows):
        lines.append("- At least one selected run is marked failed and should be treated as a failure case.")
    return lines


def stage_rows(rows, stage_name):
    folder_name = STAGE_DIR_NAMES.get(stage_name)
    return [row for row in rows if row.get("stage") == stage_name or row.get("stage_folder") == folder_name]


def write_markdown(rows, output_path, csv_path):
    created_at = datetime.now(timezone.utc).isoformat()
    lines = [
        "# Cross-Experiment Evaluation",
        "",
        f"Generated: `{created_at}`",
        "",
        "Selected landmark runs are grouped by canonical stage. Missing metrics are shown as `n/a`.",
    ]
    for stage_name in RUN_STAGES:
        folder_name = STAGE_DIR_NAMES[stage_name]
        lines.extend(["", f"## {folder_name}", ""])
        rows_for_stage = stage_rows(rows, stage_name)
        if not rows_for_stage:
            lines.append("No selected landmark run found for this stage.")
            continue
        lines.extend(
            [
                "| run_id | experiment type | completed/failed | flight time (s) | planned path (m) | actual distance (m) | min obstacle distance (m) | buffer violations | risk detections | slow_down events | replan attempts | successful replans | active replacements | final status | notes |",
                "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|",
            ]
        )
        for row in rows_for_stage:
            lines.append(
                "| "
                f"{md_value(row, 'run_id')} | "
                f"{md_value(row, 'experiment_type')} | "
                f"{md_value(row, 'completed_or_failed')} | "
                f"{md_value(row, 'total_flight_time_s')} | "
                f"{md_value(row, 'planned_path_length_m')} | "
                f"{md_value(row, 'actual_traveled_distance_m')} | "
                f"{md_value(row, 'minimum_distance_to_obstacle_m')} | "
                f"{md_value(row, 'safety_buffer_violation_count')} | "
                f"{md_value(row, 'perception_risk_detection_count')} | "
                f"{md_value(row, 'slow_down_event_count')} | "
                f"{md_value(row, 'local_replan_attempt_count')} | "
                f"{md_value(row, 'successful_local_replan_count')} | "
                f"{md_value(row, 'active_route_replacement_count')} | "
                f"{md_value(row, 'final_status')} | "
                f"{md_value(row, 'notes')} |"
            )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            *interpretation(rows),
            "",
            "## Selected Runs",
            "",
        ]
    )
    for row in rows:
        lines.append(f"- `{row['marker']}` -> `{row['run_dir']}`")
    lines.extend(["", f"CSV output: `{display_path(csv_path)}`"])
    output_path.write_text("\n".join(lines) + "\n")


def numeric_values(rows, column):
    values = [parse_float(row.get(column)) for row in rows]
    return [value for value in values if value is not None]


def mean_value(rows, column):
    values = numeric_values(rows, column)
    return statistics.mean(values) if values else None


def std_value(rows, column):
    values = numeric_values(rows, column)
    return statistics.stdev(values) if len(values) > 1 else None


def min_value(rows, column):
    values = numeric_values(rows, column)
    return min(values) if values else None


def max_value(rows, column):
    values = numeric_values(rows, column)
    return max(values) if values else None


def total_value(rows, column):
    values = numeric_values(rows, column)
    return sum(values) if values else None


def aggregate_stage_row(rows, stage_name):
    rows_for_stage = stage_rows(rows, stage_name)
    return {
        "stage": stage_name,
        "stage_folder": STAGE_DIR_NAMES[stage_name],
        "run_count": len(rows_for_stage),
        "completed_count": sum(
            1 for row in rows_for_stage if str(row.get("completed_or_failed", "")).lower() == "completed"
        ),
        "pass_count": sum(1 for row in rows_for_stage if str(row.get("final_status", "")).upper() == "PASS"),
        "mean_total_flight_time_s": mean_value(rows_for_stage, "total_flight_time_s"),
        "std_total_flight_time_s": std_value(rows_for_stage, "total_flight_time_s"),
        "min_total_flight_time_s": min_value(rows_for_stage, "total_flight_time_s"),
        "max_total_flight_time_s": max_value(rows_for_stage, "total_flight_time_s"),
        "mean_planned_path_length_m": mean_value(rows_for_stage, "planned_path_length_m"),
        "mean_actual_traveled_distance_m": mean_value(rows_for_stage, "actual_traveled_distance_m"),
        "mean_minimum_distance_to_obstacle_m": mean_value(rows_for_stage, "minimum_distance_to_obstacle_m"),
        "total_safety_buffer_violation_count": total_value(rows_for_stage, "safety_buffer_violation_count"),
        "mean_perception_risk_detection_count": mean_value(rows_for_stage, "perception_risk_detection_count"),
        "mean_slow_down_event_count": mean_value(rows_for_stage, "slow_down_event_count"),
        "mean_local_replan_attempt_count": mean_value(rows_for_stage, "local_replan_attempt_count"),
        "mean_successful_local_replan_count": mean_value(rows_for_stage, "successful_local_replan_count"),
        "mean_active_route_replacement_count": mean_value(rows_for_stage, "active_route_replacement_count"),
    }


def aggregate_rows(rows):
    return [aggregate_stage_row(rows, stage_name) for stage_name in RUN_STAGES]


def aggregate_csv_value(column, value):
    if value is None:
        return ""
    if column in {"stage", "stage_folder"}:
        return str(value)
    if column in {"run_count", "completed_count", "pass_count", "total_safety_buffer_violation_count"}:
        return str(int(value))
    return f"{float(value):.3f}"


def write_aggregate_csv(rows, output_path):
    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=AGGREGATE_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {column: aggregate_csv_value(column, row.get(column)) for column in AGGREGATE_COLUMNS}
            )


def write_included_runs_csv(rows, output_path):
    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=INCLUDED_RUN_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {column: normalized_value(column, row.get(column)) for column in INCLUDED_RUN_COLUMNS}
            )


def aggregate_md_value(row, column):
    value = aggregate_csv_value(column, row.get(column))
    return value if value != "" else "n/a"


def write_aggregate_markdown(rows, output_path, csv_path, included_runs_path, missing_stages, min_runs_per_stage):
    created_at = datetime.now(timezone.utc).isoformat()
    lines = [
        "# Aggregate Cross-Stage Comparison",
        "",
        f"Generated: `{created_at}`",
        "",
        "This summary uses all valid analyzed runs found under the four official stage folders.",
        "",
        f"Minimum requested runs per stage: `{min_runs_per_stage}`",
        "",
    ]
    if missing_stages:
        lines.extend(
            [
                "Aggregate comparison incomplete: missing required stages or not enough runs.",
                "",
                "Stages below the requested run count:",
                "",
            ]
        )
        for stage_name in missing_stages:
            lines.append(f"- `{STAGE_DIR_NAMES[stage_name]}`")
        lines.append("")
    lines.extend(
        [
            "| stage | run count | completed | pass | mean flight time (s) | std flight time (s) | min flight time (s) | max flight time (s) | mean planned path (m) | mean actual distance (m) | mean min obstacle distance (m) | total buffer violations | mean risk detections | mean slow_down events | mean replan attempts | mean successful replans | mean active replacements |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in rows:
        lines.append(
            "| "
            f"{aggregate_md_value(row, 'stage_folder')} | "
            f"{aggregate_md_value(row, 'run_count')} | "
            f"{aggregate_md_value(row, 'completed_count')} | "
            f"{aggregate_md_value(row, 'pass_count')} | "
            f"{aggregate_md_value(row, 'mean_total_flight_time_s')} | "
            f"{aggregate_md_value(row, 'std_total_flight_time_s')} | "
            f"{aggregate_md_value(row, 'min_total_flight_time_s')} | "
            f"{aggregate_md_value(row, 'max_total_flight_time_s')} | "
            f"{aggregate_md_value(row, 'mean_planned_path_length_m')} | "
            f"{aggregate_md_value(row, 'mean_actual_traveled_distance_m')} | "
            f"{aggregate_md_value(row, 'mean_minimum_distance_to_obstacle_m')} | "
            f"{aggregate_md_value(row, 'total_safety_buffer_violation_count')} | "
            f"{aggregate_md_value(row, 'mean_perception_risk_detection_count')} | "
            f"{aggregate_md_value(row, 'mean_slow_down_event_count')} | "
            f"{aggregate_md_value(row, 'mean_local_replan_attempt_count')} | "
            f"{aggregate_md_value(row, 'mean_successful_local_replan_count')} | "
            f"{aggregate_md_value(row, 'mean_active_route_replacement_count')} |"
        )
    lines.extend(
        [
            "",
            f"CSV output: `{display_path(csv_path)}`",
            f"Included runs: `{display_path(included_runs_path)}`",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n")


def write_landmark_outputs(args):
    output_dir = get_comparisons_dir() / LANDMARK_COMPARISON_NAME
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = find_selected_runs()
    csv_path = output_dir / "comparison_summary.csv"
    md_path = output_dir / "comparison_summary.md"
    selected_path = output_dir / "selected_runs.json"
    status_path = output_dir / STATUS_FILENAME

    missing_stages = missing_required_stages(rows, 1)
    if missing_stages and not args.allow_partial:
        print("Skipping landmark comparison: required stages are missing selected analyzed runs.")
        print("Missing stages:")
        for stage_name in missing_stages:
            print(f"  - {STAGE_DIR_NAMES[stage_name]}")
        print("Existing landmark comparison_summary.csv and comparison_summary.md were left unchanged.")
        write_status(
            status_path,
            rows,
            missing_stages,
            1,
            generated=False,
        )
        print(f"Wrote {display_path(status_path)}")
        return False

    write_csv(rows, csv_path)
    write_markdown(rows, md_path, csv_path)
    selected_path.write_text(json.dumps(rows, indent=2) + "\n")
    write_status(
        status_path,
        rows,
        missing_stages,
        1,
        generated=True,
    )
    print(f"Selected {len(rows)} landmark run(s).")
    print(f"Wrote {display_path(csv_path)}")
    print(f"Wrote {display_path(md_path)}")
    print(f"Wrote {display_path(selected_path)}")
    print(f"Wrote {display_path(status_path)}")
    return True


def write_aggregate_outputs(all_runs, min_runs_per_stage):
    output_dir = get_comparisons_dir() / AGGREGATE_COMPARISON_NAME
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "aggregate_summary.csv"
    md_path = output_dir / "aggregate_summary.md"
    included_runs_path = output_dir / "included_runs.csv"
    missing_stages = missing_required_stages(all_runs, min_runs_per_stage)

    if missing_stages:
        print("Aggregate comparison incomplete: missing required stages or not enough runs.")
        print("Stages below the requested run count:")
        for stage_name in missing_stages:
            print(f"  - {STAGE_DIR_NAMES[stage_name]}")

    summary_rows = aggregate_rows(all_runs)
    write_aggregate_csv(summary_rows, csv_path)
    write_aggregate_markdown(
        summary_rows,
        md_path,
        csv_path,
        included_runs_path,
        missing_stages,
        min_runs_per_stage,
    )
    write_included_runs_csv(all_runs, included_runs_path)
    print(f"Wrote {display_path(csv_path)}")
    print(f"Wrote {display_path(md_path)}")
    print(f"Wrote {display_path(included_runs_path)}")


def main():
    args = parse_args()
    if args.min_runs_per_stage < 1:
        raise ValueError("--min-runs-per-stage must be at least 1")

    ensure_output_tree()
    all_runs = find_all_valid_runs()
    print_stage_counts(all_runs)
    aggregate_missing_stages = missing_required_stages(all_runs, args.min_runs_per_stage)

    success = True
    if args.mode in {"landmark", "both"}:
        if args.mode == "both" and aggregate_missing_stages and not args.allow_partial:
            print(
                "Skipping landmark comparison refresh because the aggregate "
                "stage set is incomplete; existing landmark outputs were left unchanged."
            )
            success = False
        else:
            success = write_landmark_outputs(args) and success
    if args.mode in {"aggregate", "both"}:
        write_aggregate_outputs(all_runs, args.min_runs_per_stage)
        if aggregate_missing_stages and not args.allow_partial:
            success = False
    return 0 if success else 2


if __name__ == "__main__":
    raise SystemExit(main())
