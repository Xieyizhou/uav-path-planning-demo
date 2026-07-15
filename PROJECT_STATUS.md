# UAV Project Status

## Current Goal

Keep the repository as a GitHub-ready, resume-oriented stable demo while preserving the current PX4 SITL A* flight, perception, and replanning behavior.

Current priority: keep the public repository focused on the portfolio demo, with source, docs, curated sample outputs, raw logs, and generated experiment outputs easy to distinguish.

Deeper active replanning, dynamic obstacle handling, and waypoint target-switching research should move to a separate future repository instead of expanding this demo repo into a deep research workspace.

After the final root cleanup, README visual placeholder, and release-prep polish, the repository is ready for final `v0.1-resume-demo` release preparation.

Release-demo media tooling is local-only: `scripts/media/assemble_demo_video.py` and the reusable `scripts/media/make_demo_video.py` can assemble `release_assets/uav_path_planning_demo_preview.mp4` from existing assets, and generated release videos remain ignored by git. `make_demo_video.py` now uses a polished black minimalist portfolio style with a full roughly 14-second Gazebo segment.

## Active Workflow

1. Start PX4 SITL separately:
   `bash scripts/flight/start_px4_substation.sh`
2. Run one experiment script from `scripts/flight/experiments/`.
3. Analyze the latest log with `main.py astar analyze`.
4. Regenerate summaries with `python scripts/analysis/summarize_experiments.py`.
5. Regenerate intentional cross-stage comparisons after stage runs are analyzed:
   `python scripts/analysis/compare_experiment_sets.py --mode both --min-runs-per-stage 1`.

Detailed commands live in [docs/EXPERIMENT_PROTOCOL.md](docs/EXPERIMENT_PROTOCOL.md).

## Staged Output Rules

New outputs must be stage-scoped:

- `outputs/01_static_astar/`
- `outputs/02_perception_response/`
- `outputs/03_replan_log_only/`
- `outputs/04_active_replan/`
- `outputs/comparisons/`
- `outputs/archive/`

Do not write new `as_*` run folders directly under top-level `outputs/`. Do not compare stages by default; use `scripts/analysis/compare_experiment_sets.py` for intentional cross-stage comparisons.

## Cleaned Repository Structure

- `config/`: substation obstacle configuration.
- `src/planner/`: A* planner and obstacle-map helpers.
- `src/perception/`: simulated perception and risk-state helpers.
- `src/flight/`: MAVSDK/PX4 flight execution implementation.
- `src/logging/`: telemetry logging, analysis, summaries, plots, and comparison utilities.
- `scripts/`: runnable wrappers, PX4 launcher, and staged experiment shell scripts.
- `scripts/dev/`: development-only smoke checks and local utilities.
- `scripts/media/`: local release/demo media assembly tooling.
- `simulation/`: Gazebo world assets.
- `docs/`: durable experiment protocol, results notes, project history, and development workflow guidance.
- `data/sample_outputs/`: small curated GitHub/demo outputs.
- `data/logs/`, `data/px4_console_logs/`, `data/raw_logs/`: local raw logs; ignored by git except explanatory README files.
- `outputs/`: generated previews, run analyses, summaries, and comparisons; ignored by git except `outputs/README.md`.

## Active Entry Points

- `main.py`: CLI entry point for A* preview, flight, and analysis.
- `scripts/flight/experiments/*.sh`: formal experiment runners.
- `scripts/flight/fly_astar_path.py`: wrapper for the current flight implementation.
- `src/flight/fly_astar_path.py`: current flight implementation. Large file; refactor only in a dedicated task.
- `scripts/analysis/analyze_astar_log.py`: wrapper for per-run analysis.
- `src/logging/analyze_astar_log.py`: current per-run analysis implementation. Large file; refactor only in a dedicated task.
- `src/logging/summarize_experiments.py`: per-stage summaries and evaluation tables.
- `src/logging/compare_experiment_sets.py`: landmark and aggregate cross-stage comparisons.
- `src/logging/output_registry.py`: canonical output paths.

## Safety Rules

- Do not modify `~/PX4-Autopilot`.
- Do not delete experiment logs or real outputs.
- Do not add planning, perception, replanning, or ML features unless explicitly requested.
- Do not migrate legacy outputs unless explicitly requested.
- Keep generated caches and large outputs out of git.
- Git ignores `.venv/`, `__pycache__/`, `.pycache_compile/`, `.DS_Store`, IDE files, local env files, raw telemetry logs, PX4/Gazebo logs, and generated `outputs/` artifacts.

## Current Experiment Status

- The repo uses a four-stage experiment structure matching `outputs/01_*` through `outputs/04_*`.
- Official launcher mapping: `run_static_astar.sh`, `run_perception_response.sh`, `run_replan_log_only.sh`, and `run_active_replan.sh`.
- `scripts/flight/experiments/common.sh` is shared infrastructure, not an experiment.
- Curated landmark comparison lives in `data/sample_outputs/comparison_summary.csv`.
- Generated landmark comparison writes to `outputs/comparisons/landmark/`.
- Generated aggregate comparison writes to `outputs/comparisons/aggregate/` and summarizes all valid analyzed runs per stage.
- Full local run artifacts remain under `outputs/` and are intentionally ignored by git.
- Raw flight logs remain under `data/logs/` and are intentionally ignored by git.
- Active local replan has demonstrated route replacement in simulation, but waypoint target switching still needs validation.
- Older five-experiment wording has been removed from active docs; perception `log_only` may appear only as a diagnostic/control mode.
- Individual experiment runners update per-stage summaries only; cross-stage comparison generation is manual.
- `run_all_3x.sh` runs all official stages repeatedly, then generates both landmark and aggregate comparisons using the requested trial count as `--min-runs-per-stage`.

## Next-Step Priorities

1. Debug active replan waypoint target switching.
2. Run each staged experiment at least 3 times.
3. Add demo GIF or screenshots.
4. Add a visual architecture diagram.
5. Polish resume bullets.

## Detailed Docs

- [README.md](README.md): project overview.
- [docs/EXPERIMENT_PROTOCOL.md](docs/EXPERIMENT_PROTOCOL.md): experiment commands, outputs, and interpretation.
- [docs/architecture.md](docs/architecture.md): concise system-flow overview.
- [docs/experiment_results.md](docs/experiment_results.md): curated sample result summary.
- [docs/RELEASE_PREP.md](docs/RELEASE_PREP.md): GitHub release asset checklist.
- [docs/DEVELOPMENT_GUIDE.md](docs/DEVELOPMENT_GUIDE.md): development guide for focused maintainer workflow, file selection, refactor boundaries, and git hygiene.
- [docs/PROJECT_HISTORY.md](docs/PROJECT_HISTORY.md): historical notes.
