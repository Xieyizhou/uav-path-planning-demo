# Outputs

Generated: `2026-07-09T02:17:01.998225+00:00`

New analysis outputs are stage-scoped. Do not write new `as_*` run folders directly under `outputs/`.

## Stages

| stage | runs | summaries |
|---|---:|---|
| `static_astar` (01 static_astar) | 4 | `outputs/01_static_astar/summaries/experiment_summary.csv`, `outputs/01_static_astar/summaries/experiment_summary.md`, `outputs/01_static_astar/summaries/experiment_evaluation.csv`, `outputs/01_static_astar/summaries/experiment_evaluation.md` |
| `perception_response` (02 perception_response) | 3 | `outputs/02_perception_response/summaries/experiment_summary.csv`, `outputs/02_perception_response/summaries/experiment_summary.md`, `outputs/02_perception_response/summaries/experiment_evaluation.csv`, `outputs/02_perception_response/summaries/experiment_evaluation.md` |
| `replan_log_only` (03 replan_log_only) | 3 | `outputs/03_replan_log_only/summaries/experiment_summary.csv`, `outputs/03_replan_log_only/summaries/experiment_summary.md`, `outputs/03_replan_log_only/summaries/experiment_evaluation.csv`, `outputs/03_replan_log_only/summaries/experiment_evaluation.md` |
| `active_replan` (04 active_replan) | 3 | `outputs/04_active_replan/summaries/experiment_summary.csv`, `outputs/04_active_replan/summaries/experiment_summary.md`, `outputs/04_active_replan/summaries/experiment_evaluation.csv`, `outputs/04_active_replan/summaries/experiment_evaluation.md` |

## Comparisons

- Cross-stage comparison outputs: `outputs/comparisons`
- Use `python scripts/analysis/compare_experiment_sets.py` for intentional landmark comparisons across stages.
