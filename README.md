# Autonomous UAV Path Planning Demo

![A* grid path preview](docs/assets/grid_path.png)


Resume-oriented PX4/Gazebo project demonstrating a staged UAV autonomy pipeline: A* route planning, MAVSDK waypoint execution, simulated risk detection, slow-down response, local replanning analysis, and active route replacement.

The repository is intentionally scoped as a clean portfolio demo. It preserves the current simulation behavior and curated result artifacts while keeping raw logs, generated runs, and deeper active-replanning research out of the GitHub history.

**A short demo video is available in the [`v0.1-resume-demo`](https://github.com/Xieyizhou/uav-path-planning-replanning/releases/tag/v0.1-resume-demo) release.**

## 2-Minute Overview

**Motivation:** Low-altitude UAV inspection can reduce human exposure to risky infrastructure environments such as substations, but it requires reliable route following, obstacle awareness, and risk-response behavior. This project uses simulation to evaluate those behaviors in a repeatable way.

**System pipeline:**

```text
PX4 + Gazebo simulation
        |
MAVSDK flight control
        |git init
git branch -M main
git add .
git commit -m "Initial resume demo release"
A* global path planner
        |
Waypoint mission execution
        |
Telemetry and structured logs
        |
Simulated perception / risk detection
        |
Risk response or local replanning mode
        |
Per-stage summaries and cross-stage comparison
```

**Experiment types:**

| Stage | Launcher | Purpose |
|---|---|---|
| `01_static_astar` | `run_static_astar.sh` | Baseline A* route following without perception response. |
| `02_perception_response` | `run_perception_response.sh` | Risk detection with `slow_down` response. |
| `03_replan_log_only` | `run_replan_log_only.sh` | Generate local replan candidates without changing the active route. |
| `04_active_replan` | `run_active_replan.sh` | Replace part of the active route with a local replan. |


### Demo Results

The curated sample runs show that the staged pipeline executes end-to-end across planning, perception response, and replanning modes.

- **Static Astar**: completed the baseline route-following mission and passed the current safety checks.
- **Perception response**: recorded **305 slow-down events**, showing that simulated risk detection changes flight behavior.
- **Replan log-only**: generated **4 successful local replan candidates** without replacing the active route.
- **Active replan prototype**: recorded **1 route replacement** in simulation, confirming that the route-replacement workflow is exercised.

Active replanning remains prototype-level; dynamic obstacles and waypoint target-switching validation are reserved for a future research-focused repository.


## Repository Map

| Path | Purpose |
|---|---|
| `main.py` | CLI entry point for preview, flight, and analysis workflows. |
| `src/planner/` | A* planner and obstacle-map helpers. |
| `src/perception/` | Simulated perception and risk-state helpers. |
| `src/flight/` | MAVSDK/PX4 flight execution implementation. |
| `src/logging/` | Log analysis, summaries, plots, and comparisons. |
| `scripts/flight/experiments/` | Four official experiment launchers and shared shell helpers. |
| `scripts/analysis/` | Analysis wrappers and comparison scripts. |
| `scripts/dev/` | Development-only smoke checks and local utilities. |
| `docs/` | Architecture, protocol, result notes, and release-prep docs. |
| `data/sample_outputs/` | Small curated sample outputs committed for GitHub readers. |
| `outputs/` | Local generated outputs; ignored by git except `outputs/README.md`. |

## How To Run The Demo

Prerequisites: Python 3, PX4 SITL/Gazebo configured locally, and MAVSDK dependencies installed. This repository does not modify `~/PX4-Autopilot`.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Start PX4/Gazebo in one terminal:

```bash
bash scripts/flight/start_px4_substation.sh
```

Run one staged experiment in a second terminal:

```bash
source .venv/bin/activate
bash scripts/flight/experiments/run_static_astar.sh
```

Other official stages:

```bash
bash scripts/flight/experiments/run_perception_response.sh
bash scripts/flight/experiments/run_replan_log_only.sh
bash scripts/flight/experiments/run_active_replan.sh
```

Run all four stages repeatedly and regenerate comparisons:

```bash
bash scripts/flight/experiments/run_all_3x.sh
```

Regenerate cross-stage comparison only:

```bash
python scripts/analysis/compare_experiment_sets.py --mode both --min-runs-per-stage 3
```

## Sample Outputs

Curated, GitHub-safe sample outputs live in `data/sample_outputs/`:

- `comparison_summary.csv`: landmark cross-stage comparison.
- `comparison_summary.md`: markdown rendering of the same comparison.
- `selected_runs.json`: metadata for the selected landmark runs.

Full local runs, plots, summaries, and comparison outputs are generated under `outputs/`. Raw telemetry logs are generated under `data/logs/`. These directories are intentionally ignored so large logs, temporary files, videos, and regenerated artifacts do not enter the portfolio repo.

See [docs/experiment_results.md](docs/experiment_results.md) for result interpretation and [docs/RELEASE_PREP.md](docs/RELEASE_PREP.md) for GitHub release packaging.

## Current Scope

This is the stable resume/demo version of the project. It is meant to show a working autonomy pipeline, experiment discipline, logging, and comparison tooling in a compact repository.

The deeper research thread, especially active replanning robustness, dynamic obstacle handling, and waypoint target-switching validation, should move to a separate future repository so this project stays focused and easy to review.

## Limitations

- Simulation only; no real UAV hardware validation.
- Perception is rule-based/simulated rather than camera-based deep perception.
- Active local replanning is demonstrated but still needs deeper target-switching validation.
- Current results are from one simulated substation-style environment.
