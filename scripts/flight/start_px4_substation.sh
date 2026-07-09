#!/usr/bin/env bash
# Launch PX4 SITL with this repository's substation Gazebo world.
#
# The script copies the local SDF world into the PX4 checkout and starts
# `make px4_sitl gz_x500`. It does not modify the flight code in PX4; it only
# provides the world file required by this project's experiments.
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-$HOME/projects/drone-ai}"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
WORLD_NAME="${WORLD_NAME:-substation_simple}"

WORLD_SRC="$PROJECT_ROOT/simulation/worlds/${WORLD_NAME}.sdf"
WORLD_DST="$PX4_ROOT/Tools/simulation/gz/worlds/${WORLD_NAME}.sdf"

echo "===================================="
echo "Starting PX4 + Gazebo Substation SITL"
echo "===================================="
echo "Project root: $PROJECT_ROOT"
echo "PX4 root:     $PX4_ROOT"
echo "World:        $WORLD_NAME"
echo

if [[ ! -f "$WORLD_SRC" ]]; then
  echo "ERROR: world file not found:"
  echo "$WORLD_SRC"
  exit 1
fi

echo "Copying world file into PX4 Gazebo worlds folder..."
mkdir -p "$(dirname "$WORLD_DST")"
cp "$WORLD_SRC" "$WORLD_DST"

echo "Stopping old PX4/Gazebo processes if any..."
pkill -f "px4" || true
pkill -f "gz sim" || true
pkill -f "gz" || true

sleep 2

cd "$PX4_ROOT"

if [[ -f ".venv/bin/activate" ]]; then
  echo "Activating PX4 virtual environment..."
  source .venv/bin/activate
else
  echo "PX4 .venv not found, continuing without activating it."
fi

if command -v brew >/dev/null 2>&1 && brew --prefix opencv >/dev/null 2>&1; then
  export OpenCV_DIR="$(brew --prefix opencv)/lib/cmake/opencv4"
  echo "OpenCV_DIR=$OpenCV_DIR"
fi

echo
echo "Launching PX4 SITL..."
echo "Do not close this terminal while flying."
echo

PX4_GZ_WORLD="$WORLD_NAME" make px4_sitl gz_x500
