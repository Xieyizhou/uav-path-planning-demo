# Experiment Results

This page summarizes the current curated landmark comparison. The source files are:

- `data/sample_outputs/comparison_summary.csv`
- `data/sample_outputs/comparison_summary.md`
- `data/sample_outputs/selected_runs.json`

The full generated output tree remains local under `outputs/`.

Cross-stage comparison outputs are generated intentionally, not after every individual experiment. Running a single experiment updates that stage's summaries but does not overwrite cross-stage comparisons.

There are now two comparison layers:

- **Landmark comparison**: selects one representative run per stage and writes `outputs/comparisons/landmark/comparison_summary.csv`. This is useful for README/demo presentation.
- **Aggregate comparison**: summarizes all valid analyzed runs per stage and writes `outputs/comparisons/aggregate/aggregate_summary.csv` plus `included_runs.csv`. This is the correct repeated-trial output after `run_all_3x.sh`.

A four-row `comparison_summary.csv` is not wrong; it is the landmark comparison, not the repeated-trial statistical summary.

## Four-Stage Experiment Design

The formal experiment pipeline has four stages, matching `outputs/01_*` through `outputs/04_*`:

| Experiment | Output Stage | Launcher | Meaning |
|---:|---|---|---|
| 1 | `01_static_astar` | `run_static_astar.sh` | Baseline A* path following |
| 2 | `02_perception_response` | `run_perception_response.sh` | Perception-enabled risk response using `slow_down` |
| 3 | `03_replan_log_only` | `run_replan_log_only.sh` | Generate and log local replan candidates without route replacement |
| 4 | `04_active_replan` | `run_active_replan.sh` | Active local route replacement |

Older or diagnostic perception `log_only` runs may exist in local outputs, but they are not a separate formal experiment in the current four-stage design.

## Landmark Comparison

| experiment | output stage | mode | run id | flight time (s) | planned path (m) | actual distance (m) | min obstacle distance (m) | risk detections | slow_down events | replan attempts | successful replans | active replacements | status |
|---:|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | `01_static_astar` | baseline A* | `as_20260706_092030` | 149.132 | 68.000 | 67.135 | n/a | 0 | 0 | 0 | 0 | 0 | PASS |
| 2 | `02_perception_response` | perception_response (`slow_down` action) | `as_20260707_051714` | 177.210 | 68.000 | 67.093 | 0.843 | 679 | 305 | 0 | 0 | 0 | PASS |
| 3 | `03_replan_log_only` | replan log-only | `as_20260707_055516` | 149.169 | 68.000 | 67.256 | 0.883 | 541 | 0 | 4 | 4 | 0 | PASS |
| 4 | `04_active_replan` | active replan | `as_20260707_050318` | 220.088 | 82.000 | 70.520 | 0.823 | 865 | 0 | 3 | 3 | 1 | PASS |

## Aggregate Comparison

Generate the aggregate repeated-trial summary after staged runs are available:

```bash
python scripts/analysis/compare_experiment_sets.py --mode aggregate --min-runs-per-stage 3
```

This creates:

- `outputs/comparisons/aggregate/aggregate_summary.csv`
- `outputs/comparisons/aggregate/aggregate_summary.md`
- `outputs/comparisons/aggregate/included_runs.csv`

If a stage has fewer than the requested run count, the command still writes the aggregate files and records a warning so the incomplete stage set is visible.

## Interpretation

- Baseline A* completed the route without perception risk logging.
- The `perception_response` stage produced 305 slow-down events and increased flight time relative to the static baseline while keeping the planned route unchanged.
- Replan log-only found successful local replan candidates four times while leaving the active route unchanged.
- Active replan recorded one route replacement, confirming that the active replacement path is exercised.
- All selected landmark runs are marked `PASS`, with zero safety-buffer violations in the comparison table.

## Known Issues and Next Steps

- Active local replan still needs waypoint target-switching validation. The selected active run notes: `Target jumped from RWP06 to WP09 at t=90.30s; check target switching.`
- Several runs include near-boundary clearance warnings even when they do not enter raw obstacle footprints or inflated safety buffers.
- Run each of the four staged experiments at least three times before treating the metrics as stable benchmark results.
- Add a system architecture diagram or short demo GIF for GitHub readers.
