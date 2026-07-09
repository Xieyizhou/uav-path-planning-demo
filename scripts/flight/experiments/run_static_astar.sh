#!/usr/bin/env bash
# Formal stage 01 runner: static_astar.
#
# Perception and local replanning are disabled. This run is the control case
# for comparing later perception and replanning behavior.
set -euo pipefail

source "$(dirname "$0")/common.sh"

echo "===================================="
echo "Experiment 1: static_astar"
echo "Output stage: 01_static_astar"
echo "Perception: disabled"
echo "Risk action: none"
echo "Replan: disabled"
echo "===================================="

cleanup_mavsdk

python main.py astar fly \
  --obstacle-config "$OBSTACLE_CONFIG" \
  --return-home \
  --altitude 1.5 \
  --max-speed 0.8 \
  --return-speed-scale 0.7 \
  --waypoint-acceptance 0.3

analyze_latest
mark_latest_output "static_astar" "KEEP_BASELINE__astar_only_perception_disabled.txt"
update_stage_reports
