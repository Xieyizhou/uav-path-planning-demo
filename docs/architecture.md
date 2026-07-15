# Architecture

This project is a simulation-first UAV autonomy pipeline. The core source code
lives in `src/`; scripts under `scripts/` are runnable wrappers and experiment
launchers.

## System Flow

```text
PX4/Gazebo simulation
  -> Unified command layer (`main.py` / `src/cli/`)
  -> Flight implementation (`src/flight/fly_astar_path.py`)
  -> A* global planner (`src/planner/`)
  -> Local NED waypoint execution
  -> Telemetry CSV logging (`src/logging/flight_logger.py`)
  -> Simulated perception/risk state (`src/perception/`)
  -> Risk action
       - Experiment 2 perception_response: reduce speed during warning/danger risk
       - Experiment 3 replan log-only: test replan availability without replacing route
       - Experiment 4 active route replacement: replace remaining outbound waypoints
  -> Per-run analysis and staged summaries (`src/logging/`)
  -> Curated sample comparison (`data/sample_outputs/comparison_summary.csv`)
```

## Major Modules

- `src/planner/`: grid A* search, path simplification, obstacle-map conversion.
- `src/perception/`: map-based simulated obstacle detector and structured risk state.
- `src/flight/`: PX4/MAVSDK flight execution, waypoint tracking, perception response, and local replan orchestration.
- `src/logging/`: telemetry CSV formatting, log loading, metrics, plotting, summaries, and comparison reports.
- `scripts/flight/experiments/`: repeatable staged experiment runners.
- `config/substation_obstacles.json`: substation obstacle map used by planning and simulated perception.

## Formal Experiment Runners

| Experiment | Output Stage | Runner |
|---:|---|---|
| 1 | `01_static_astar` | `scripts/flight/experiments/run_static_astar.sh` |
| 2 | `02_perception_response` | `scripts/flight/experiments/run_perception_response.sh` |
| 3 | `03_replan_log_only` | `scripts/flight/experiments/run_replan_log_only.sh` |
| 4 | `04_active_replan` | `scripts/flight/experiments/run_active_replan.sh` |

`scripts/flight/experiments/common.sh` is shared runner infrastructure, not an experiment.

## Runtime Boundaries

PX4 SITL is started separately by `scripts/flight/start_px4_substation.sh`.
Flight commands connect through MAVSDK and do not modify PX4 flight code.

Generated raw logs and full experiment outputs remain local:

- `data/logs/`
- `data/px4_console_logs/`
- `outputs/`

Small GitHub-ready sample outputs are copied to `data/sample_outputs/`.
