# UAV Project Status

## Current Goal

Maintain this repository as a stable, resume-oriented PX4/Gazebo autonomy demo.
The current release target is `v0.2.0`, covering the unified command center,
five-map test catalog, destination presets, hardened runtime behavior, and
validated active-replan target switching.

The demo intentionally remains simulation-first. Dynamic obstacles, real sensor
perception, hardware flight, and broader research experiments are future work.

## Active Workflow

Run public commands through `main.py` from the repository root:

```bash
source .venv/bin/activate
python main.py map
python main.py point
python main.py map start
```

In a second terminal, choose a compact task or official experiment:

```bash
python main.py task list
python main.py task run fly_round_trip
python main.py experiment list
python main.py experiment run static
```

Reports and checks are also integrated:

```bash
python main.py report summarize
python main.py report compare --mode both --min-runs-per-stage 3
python main.py report validate-active --latest 3
python main.py check all
```

See [docs/CLI_REFERENCE.md](docs/CLI_REFERENCE.md) for the complete command set.

## Repository Structure

- `main.py` and `src/cli/`: unified user command layer.
- `src/planner/`: A* search and obstacle-map conversion.
- `src/perception/`: simulated obstacle detection and risk states.
- `src/flight/`: MAVSDK/PX4 flight execution, tasks, and replanning.
- `src/maps/`: map catalog, destination persistence, and goal-marker sync.
- `src/logging/`: telemetry, analysis, summaries, and comparisons.
- `scripts/flight/experiments/`: internal four-stage experiment launchers.
- `scripts/maps/`: map generation and manager implementations.
- `simulation/worlds/`: five Gazebo test environments.
- `config/maps/`: matching A* configurations and map catalog.
- `data/sample_outputs/`: small curated landmark and aggregate results.
- `outputs/`: local generated runs and reports; ignored except its README.

## Safety and Reliability

- Connection, position, telemetry, waypoint, landing, and logger timeouts.
- Explicit non-zero failure propagation through the CLI.
- Confirmed landing state and atomic run-status records.
- Cleanup limited to project-managed flight and PX4 PIDs.
- Map switching blocked while a managed flight or PX4 session is active.
- Parameter, map, destination, and A* reachability validation.
- 62 passing offline tests plus shell and preview checks in CI.

## Experiment Status

The four official stages currently have these analyzed runs:

| Stage | Runs | Completed | PASS |
| --- | ---: | ---: | ---: |
| Static A* | 4 | 4 | 4 |
| Perception response | 3 | 3 | 3 |
| Replan log-only | 3 | 3 | 3 |
| Active replan | 6 | 6 | 6 |

The latest three eligible active-replan runs all pass strict target-switching
validation. Each records the contiguous outbound sequence
`RWP01 → RWP02 → RWP03 → RWP04 → RWP05 → RWP06`, no old `WP` target after
replacement, original-goal arrival, and completed landing.

The refreshed public landmark uses active run `as_20260713_070842`. The public
aggregate includes all 16 valid analyzed runs and records zero safety-buffer
violations across all four stages.

## Remaining Evidence Gaps

- Formal experiment manifests currently come from `substation_simple_v3`.
- The other four maps are validated offline but still need representative
  PX4/Gazebo flight runs.
- Perception remains rule-based and map-based rather than sensor-driven.
- The project has no real-airframe validation or dynamic-obstacle benchmark.
- Several large flight and reporting modules remain candidates for refactoring.

## Next Priorities

1. Run representative PX4/Gazebo missions on training, medium, complex, and
   extreme maps.
2. Exercise active replanning on complex and extreme maps.
3. Pin Python dependency versions and add lint/type/coverage checks.
4. Split the largest flight and report modules without changing behavior.
5. Keep future hardware and dynamic-obstacle work in a separate research scope.

## Release State

- `v0.1-demo`: original public demo release.
- `v0.2.0`: current resume-demo release target.
- License: MIT.
- Generated videos, raw telemetry, simulator logs, and full output trees remain
  outside Git history.

## Detailed Documentation

- [README.md](README.md): portfolio overview and measured results.
- [docs/EXPERIMENT_PROTOCOL.md](docs/EXPERIMENT_PROTOCOL.md): experiment protocol.
- [docs/MAP_TESTING.md](docs/MAP_TESTING.md): map and destination workflow.
- [docs/experiment_results.md](docs/experiment_results.md): current evidence.
- [docs/RELEASE_PREP.md](docs/RELEASE_PREP.md): release checklist.
