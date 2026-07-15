#!/usr/bin/env bash
# Launch PX4 SITL with this repository's substation Gazebo world.
#
# The script copies the local SDF world into the PX4 checkout and starts
# `make px4_sitl gz_x500`. It does not modify the flight code in PX4; it only
# provides the world file required by this project's experiments.
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-$HOME/projects/drone-ai}"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
MAP_SWITCHER="$PROJECT_ROOT/scripts/maps/switch_map.py"
TARGET_PREPARER="$PROJECT_ROOT/scripts/maps/prepare_selected_world.py"
MAP_PYTHON="$PROJECT_ROOT/.venv/bin/python"
if [[ ! -x "$MAP_PYTHON" ]]; then
  MAP_PYTHON="$(command -v python3)"
fi

if [[ -z "${WORLD_NAME+x}" ]] && [[ -f "$MAP_SWITCHER" ]]; then
  MAP_ID="$("$MAP_PYTHON" "$MAP_SWITCHER" current --field id)"
  WORLD_NAME="$("$MAP_PYTHON" "$MAP_SWITCHER" current --field world_name)"
  WORLD_RELATIVE_PATH="$("$MAP_PYTHON" "$MAP_SWITCHER" current --field world_file)"
  WORLD_SRC="$PROJECT_ROOT/$WORLD_RELATIVE_PATH"
  MODEL_SPAWN_POSE="$("$MAP_PYTHON" "$MAP_SWITCHER" current --field spawn_pose)"
else
  MAP_ID="${MAP_ID:-custom}"
  WORLD_NAME="${WORLD_NAME:-substation_simple}"
  WORLD_SRC="${WORLD_SRC:-$PROJECT_ROOT/simulation/worlds/${WORLD_NAME}.sdf}"
  MODEL_SPAWN_POSE="${PX4_GZ_MODEL_POSE:--10,-10,0,0,0,0}"
fi

WORLD_DST="$PX4_ROOT/Tools/simulation/gz/worlds/${WORLD_NAME}.sdf"
RUNTIME_DIR="$PROJECT_ROOT/.runtime"
PX4_PID_FILE="$RUNTIME_DIR/px4_launcher.pid"

echo "===================================="
echo "Starting PX4 + Gazebo Test Map"
echo "===================================="
echo "Project root: $PROJECT_ROOT"
echo "PX4 root:     $PX4_ROOT"
echo "Map ID:       $MAP_ID"
echo "World:        $WORLD_NAME"
echo "World file:   $WORLD_SRC"
echo "Model spawn:  $MODEL_SPAWN_POSE"
echo

if [[ ! -f "$WORLD_SRC" ]]; then
  echo "ERROR: world file not found:"
  echo "$WORLD_SRC"
  exit 1
fi

mkdir -p "$RUNTIME_DIR"
if [[ -f "$PX4_PID_FILE" ]]; then
  previous_pid="$(cat "$PX4_PID_FILE")"
  if [[ "$previous_pid" =~ ^[0-9]+$ ]] && kill -0 "$previous_pid" 2>/dev/null; then
    previous_command="$(ps -p "$previous_pid" -o command= 2>/dev/null || true)"
    if [[ "$previous_command" == *"start_px4_substation.sh"* ]]; then
      echo "ERROR: this project already has a PX4 launcher running as PID $previous_pid."
      echo "Stop that launcher before starting another one."
      exit 1
    fi
  fi
  rm -f "$PX4_PID_FILE"
fi

# Gazebo Sim starts its server as an independent process. If the terminal or
# PX4 launcher is killed abruptly, that server can survive after the PID file
# is removed and prevent the next launch from loading this world correctly.
# Only clean servers whose command line references this project's exact world.
stale_gazebo_pids=()
while IFS= read -r candidate_pid; do
  [[ "$candidate_pid" =~ ^[0-9]+$ ]] || continue
  candidate_command="$(ps -p "$candidate_pid" -o command= 2>/dev/null || true)"
  if [[ "$candidate_command" == *"gz sim"* ]] && [[ "$candidate_command" == *"$WORLD_DST"* ]]; then
    stale_gazebo_pids+=("$candidate_pid")
  fi
done < <(pgrep -f "$WORLD_DST" 2>/dev/null || true)

if (( ${#stale_gazebo_pids[@]} > 0 )); then
  echo "Cleaning stale Gazebo server(s) for $WORLD_NAME: ${stale_gazebo_pids[*]}"
  for stale_pid in "${stale_gazebo_pids[@]}"; do
    kill -TERM "$stale_pid" 2>/dev/null || true
  done

  for _ in {1..50}; do
    servers_still_running=false
    for stale_pid in "${stale_gazebo_pids[@]}"; do
      if kill -0 "$stale_pid" 2>/dev/null; then
        servers_still_running=true
        break
      fi
    done
    [[ "$servers_still_running" == false ]] && break
    sleep 0.1
  done

  for stale_pid in "${stale_gazebo_pids[@]}"; do
    if kill -0 "$stale_pid" 2>/dev/null; then
      echo "ERROR: stale Gazebo server PID $stale_pid did not stop."
      echo "Stop that process before starting PX4 again."
      exit 1
    fi
  done
fi

echo "Copying world file into PX4 Gazebo worlds folder..."
mkdir -p "$(dirname "$WORLD_DST")"
WORLD_COPY_SRC="$WORLD_SRC"
if [[ "$MAP_ID" != "custom" ]] && [[ -f "$TARGET_PREPARER" ]]; then
  RUNTIME_WORLD="$RUNTIME_DIR/worlds/${WORLD_NAME}.sdf"
  "$MAP_PYTHON" "$TARGET_PREPARER" \
    --map "$MAP_ID" \
    --source "$WORLD_SRC" \
    --output "$RUNTIME_WORLD"
  WORLD_COPY_SRC="$RUNTIME_WORLD"
fi
cp "$WORLD_COPY_SRC" "$WORLD_DST"

printf '%s\n' "$$" > "$PX4_PID_FILE"
cleanup_pid_file() {
  if [[ -f "$PX4_PID_FILE" ]] && [[ "$(cat "$PX4_PID_FILE")" == "$$" ]]; then
    rm -f "$PX4_PID_FILE"
  fi
}
trap cleanup_pid_file EXIT

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

export PX4_GZ_MODEL_POSE="$MODEL_SPAWN_POSE"
PX4_GZ_WORLD="$WORLD_NAME" make px4_sitl gz_x500
