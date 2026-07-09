#!/usr/bin/env bash
# Formal stage 02 runner: perception_response.
#
# The route remains the original A* path; only speed is reduced during
# warning/danger risk. This isolates behavior change from replanning.
set -euo pipefail

source "$(dirname "$0")/common.sh"

echo "===================================="
echo "Experiment 2: perception_response"
echo "Output stage: 02_perception_response"
echo "Perception: enabled"
echo "Risk action: slow_down"
echo "Replan: disabled"
echo "===================================="

cleanup_mavsdk

python main.py astar fly \
  --obstacle-config "$OBSTACLE_CONFIG" \
  --return-home \
  --altitude 1.5 \
  --max-speed 0.8 \
  --return-speed-scale 0.7 \
  --waypoint-acceptance 0.3 \
  --enable-perception \
  --risk-action slow_down \
  --waypoint-timeout auto \
  --min-risk-speed 0.3

analyze_latest
mark_latest_output "perception_response" "KEEP_COMPARISON__perception_slow_down.txt"
update_stage_reports
