# Cross-Experiment Evaluation

Generated: `2026-07-07T07:18:09.838511+00:00`

Selected landmark runs are grouped by canonical stage. Missing metrics are shown as `n/a`.

## 01_static_astar

| run_id | experiment type | completed/failed | flight time (s) | planned path (m) | actual distance (m) | min obstacle distance (m) | buffer violations | risk detections | slow_down events | replan attempts | successful replans | active replacements | final status | notes |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| as_20260706_092030 | astar_only_perception_disabled | completed | 149.132 | 68.000 | 67.135 | n/a | 0 | 0 | 0 | 0 | 0 | 0 | PASS | Near-boundary clearance warning: actual trajectory came within about 0.84 m of an obstacle cell center. |

## 02_perception_response

| run_id | experiment type | completed/failed | flight time (s) | planned path (m) | actual distance (m) | min obstacle distance (m) | buffer violations | risk detections | slow_down events | replan attempts | successful replans | active replacements | final status | notes |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| as_20260707_051714 | perception_slow_down | completed | 177.210 | 68.000 | 67.093 | 0.843 | 0 | 679 | 305 | 0 | 0 | 0 | PASS | Near-boundary clearance warning: actual trajectory came within about 0.81 m of an obstacle cell center. |

## 03_replan_log_only

| run_id | experiment type | completed/failed | flight time (s) | planned path (m) | actual distance (m) | min obstacle distance (m) | buffer violations | risk detections | slow_down events | replan attempts | successful replans | active replacements | final status | notes |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| as_20260707_055516 | local_replan_log_only | completed | 149.169 | 68.000 | 67.256 | 0.883 | 0 | 541 | 0 | 4 | 4 | 0 | PASS | Near-boundary clearance warning: actual trajectory came within about 0.88 m of an obstacle cell center. |

## 04_active_replan

| run_id | experiment type | completed/failed | flight time (s) | planned path (m) | actual distance (m) | min obstacle distance (m) | buffer violations | risk detections | slow_down events | replan attempts | successful replans | active replacements | final status | notes |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| as_20260707_050318 | active_local_replan | completed | 220.088 | 82.000 | 70.520 | 0.823 | 0 | 865 | 0 | 3 | 3 | 1 | PASS | Target jumped from RWP06 to WP09 at t=90.30s; check target switching.; Near-boundary clearance warning: actual trajectory came within about 0.76 m of an obstacle cell center. |

## Interpretation

- This is an intentional cross-stage comparison; ordinary stage summaries remain separate.
- The active replan landmark recorded route replacement, so it changed the outbound route rather than only logging a possible replan.

## Selected Runs

- `KEEP_BASELINE__astar_only_perception_disabled.txt` -> `outputs/01_static_astar/runs/as_20260706_092030`
- `KEEP_COMPARISON__perception_slow_down.txt` -> `outputs/02_perception_response/runs/as_20260707_051714`
- `KEEP_CONTROL__log_only_local_replan_no_route_replacement.txt` -> `outputs/03_replan_log_only/runs/as_20260707_055516`
- `KEEP_LANDMARK__active_local_replan.txt` -> `outputs/04_active_replan/runs/as_20260707_050318`

CSV output: `outputs/comparisons/landmark/comparison_summary.csv`
