# Development Guide

This guide keeps future maintenance work focused, reviewable, and low-noise.

## Read First

For most tasks, read these first:

1. `PROJECT_STATUS.md`
2. `README.md`
3. `docs/EXPERIMENT_PROTOCOL.md` when the task mentions experiments, outputs, summaries, or comparisons
4. This file when the task mentions refactors, project hygiene, or development workflow

Then inspect only the files named by the task unless local context proves another file is necessary.

## Choosing Relevant Files

- Experiment commands and interpretation: `docs/EXPERIMENT_PROTOCOL.md`, `scripts/flight/experiments/`
- Output paths and stage layout: `src/logging/output_registry.py`, `outputs/README.md`
- Per-run analysis: `src/logging/analyze_astar_log.py`
- Per-stage summaries: `src/logging/summarize_experiments.py`
- Cross-stage comparison: `src/logging/compare_experiment_sets.py`
- Flight behavior: `src/flight/fly_astar_path.py`
- Runnable compatibility wrappers: `scripts/analysis/`, `scripts/flight/`

Do not inspect broad directories by default. Prefer `rg` and targeted file reads.

## Large Files To Handle Carefully

- `src/flight/fly_astar_path.py` is large and currently owns waypoint control, perception response, and local replan runtime.
- `src/logging/analyze_astar_log.py` is large and currently owns log loading, metrics, plotting, collision checks, report writing, and manifest writing.

For small tasks, avoid sweeping edits in these files. For refactor tasks, make behavior-preserving changes with focused verification.

## Future Refactor Plan

Do not perform this split unless explicitly requested.

- Continue splitting `src/logging/analyze_astar_log.py` into log loading, metrics, plotting, collision checks, and report writing modules.
- Continue splitting `src/flight/fly_astar_path.py` into flight config, telemetry logging, waypoint control, replan runtime, and preview writing modules.
- Extract shared evaluation columns and run collection logic from `summarize_experiments.py` and `compare_experiment_sets.py` into shared analysis modules.
- Keep `main.py` as a thin command-line entry point.

Suggested sequence:

1. Add characterization tests or fixture-based checks for current analysis outputs.
2. Extract pure parsing and metric functions first.
3. Extract report-writing modules after metrics are stable.
4. Extract flight-side helpers only after the analysis refactor is stable.
5. Keep each refactor PR behavior-preserving.

## What Not To Touch

- Do not modify `~/PX4-Autopilot`.
- Do not delete experiment logs or real outputs.
- Do not migrate legacy outputs unless explicitly requested.
- Do not add planning, perception, replanning, or ML features during hygiene/refactor-prep tasks.
- Do not change experiment runner behavior unless the task explicitly asks for it.

## Git Hygiene

Do not commit generated caches or local machine files:

- `.DS_Store`
- `.venv/`
- `__pycache__/`
- `.pycache_compile/`
- `*.pyc`, `*.pyo`, `*.pyd`
- `.idea/`
- `.env`
- `data/logs/`
- `data/px4_console_logs/`
- generated contents under `outputs/`

`outputs/README.md` is intentionally allowed so the staged output contract can be documented without committing generated run data.

Before committing, check:

```bash
git status --short
git diff --stat
```

Usually commit code and documentation only. Commit generated outputs only when they are small, intentional examples.

## Recommended Maintainer Task Template

```text
You are working inside ~/projects/drone-ai.

Read PROJECT_STATUS.md first.

Task:
<specific objective>

Relevant files only:
<explicit list>

Constraints:
- Do not modify ~/PX4-Autopilot.
- Do not delete logs or outputs.
- Do not change flight behavior unless requested.
- Do not add planning/perception/replanning/ML features unless requested.

Validation:
<specific commands to run>

Report:
<exact final report fields>
```
