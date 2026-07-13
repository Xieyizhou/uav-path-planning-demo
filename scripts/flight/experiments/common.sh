#!/usr/bin/env bash
# Shared helpers for the formal staged experiment runners.
#
# Each runner sources this file after PX4 SITL has been started separately.
# The helpers activate the project virtual environment, clean stale MAVSDK
# processes, analyze the latest flight log, mark the corresponding staged
# output folder, and refresh per-stage reports.
set -euo pipefail

if [[ -z "${PROJECT_ROOT+x}" ]]; then
  COMMON_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  PROJECT_ROOT="$(cd -- "$COMMON_DIR/../../.." && pwd)"
fi
OBSTACLE_CONFIG="${OBSTACLE_CONFIG:-config/substation_obstacles.json}"

cd "$PROJECT_ROOT"

if [[ -f ".venv/bin/activate" ]]; then
  source .venv/bin/activate
else
  echo "ERROR: .venv not found at $PROJECT_ROOT/.venv"
  exit 1
fi

cleanup_mavsdk() {
  # Stale mavsdk_server or flight-script processes can keep UDP ports busy and
  # make the next PX4 connection ambiguous, so each formal run starts clean.
  echo "Cleaning old MAVSDK / flight processes..."
  pkill -f "mavsdk_server" || true
  pkill -f "scripts/flight/fly_astar_path.py" || true
  sleep 2
}

analyze_latest() {
  echo
  echo "Analyzing latest A* log..."
  python main.py astar analyze --obstacle-config "$OBSTACLE_CONFIG"
}

update_stage_reports() {
  echo
  echo "Updating per-stage experiment summaries..."
  python scripts/analysis/summarize_experiments.py
}

update_cross_stage_comparison() {
  echo
  echo "Updating landmark cross-stage comparison..."
  python scripts/analysis/compare_experiment_sets.py
}

latest_output_dir() {
  # Stage names map to the canonical output folders documented in
  # docs/EXPERIMENT_PROTOCOL.md.
  local stage="$1"
  local stage_dir
  case "$stage" in
    static_astar) stage_dir="outputs/01_static_astar/runs" ;;
    perception_response) stage_dir="outputs/02_perception_response/runs" ;;
    replan_log_only) stage_dir="outputs/03_replan_log_only/runs" ;;
    active_replan) stage_dir="outputs/04_active_replan/runs" ;;
    *)
      echo "ERROR: unknown output stage: $stage" >&2
      return 1
      ;;
  esac
  find "$stage_dir" -maxdepth 1 -type d -name 'as_[0-9]*' | sort | tail -1
}

mark_latest_output() {
  local stage="$1"
  local marker_name="$2"
  local latest
  latest="$(latest_output_dir "$stage")"

  if [[ -z "$latest" ]]; then
    echo "WARNING: no $stage as_[timestamp] folder found to mark."
    return 0
  fi

  touch "$latest/$marker_name"
  echo
  echo "Marked output:"
  echo "  $latest/$marker_name"
}
