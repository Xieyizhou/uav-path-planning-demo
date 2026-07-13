# Changelog

This file records the major technical milestones and user-visible changes in the UAV path-planning demo. Routine command examples and current operating instructions belong in `README.md` and `docs/EXPERIMENT_PROTOCOL.md`.

## v0.1-demo — 2026-07-09

### Flight foundations

- Added PX4 SITL and Gazebo integration through MAVSDK-Python.
- Added connection, health, arming, takeoff, hover, landing, and telemetry logging workflows.
- Added an Offboard square-flight experiment with timestamped CSV telemetry and diagnostic plots.
- Added closed-loop local NED waypoint following using position feedback and bounded velocity commands.
- Added waypoint acceptance checks, per-waypoint timeout handling, and return-to-start execution.

### Global planning and simulation environment

- Added a grid-based A* planner with optional diagonal movement and path simplification.
- Added conversion from grid-cell centers to local NED waypoints.
- Added dry-run route previews and JSON path exports.
- Added a simplified substation-style Gazebo world with transformers, cabinets, poles, busbars, service corridors, and map-reference markers.
- Added a shared obstacle configuration used by route planning, simulation validation, and simulated perception.
- Added 2.5D obstacle filtering based on flight altitude and obstacle height.
- Separated raw physical obstacle footprints from horizontally inflated planning keep-out cells.
- Added map-quality checks for excessive occupancy, boundary-hugging routes, goal adjacency, and route-clearance violations.

### Perception and risk response

- Added map-based simulated forward obstacle detection with configurable range and field of view.
- Added structured risk states: `clear`, `detected`, `warning`, and `danger`.
- Added perception and nearest-obstacle fields to flight telemetry logs.
- Added configurable risk actions including logging, speed reduction, and emergency landing behavior.
- Added adaptive waypoint timeout estimation and a minimum commanded-speed floor for slowdown runs.

### Local replanning

- Added local A* replanning triggered by configured perception-risk levels.
- Added a log-only mode that records candidate replans without changing the active route.
- Added an active mode that can replace the remaining outbound route with a local replan.
- Added replan attempt, success, route-replacement, and replanned-waypoint fields to logs and reports.

### Experiment and analysis workflow

- Added four formal experiment stages:
  - `01_static_astar`
  - `02_perception_response`
  - `03_replan_log_only`
  - `04_active_replan`
- Added reusable experiment launchers and a repeated-run workflow.
- Added stage-scoped output directories for runs, previews, summaries, and comparisons.
- Added per-run summaries, manifests, run metadata, trajectory plots, tracking-error plots, and perception timelines.
- Added collision and clearance checks that distinguish raw-footprint entries from inflated safety-buffer entries.
- Added per-stage evaluation tables and cross-stage landmark and aggregate comparisons.
- Added curated sample outputs for quick inspection without committing raw telemetry or complete generated output trees.

### Project structure and documentation

- Reorganized implementation code under `src/` and kept runnable commands under `scripts/`.
- Added a unified command-line entry point for preview, flight, and analysis workflows.
- Added architecture, experiment protocol, result interpretation, simulation, and release-media documentation.
- Added a short simulation demo video as a release asset.

### Notable corrections

- Corrected Gazebo floor and object alignment using consistent center-height placement.
- Revised obstacle footprints after the inflated map over-blocked internal corridors.
- Improved visual correspondence between the Gazebo environment and the A* planning grid.
- Preserved raw logs and generated run folders instead of deleting older experiment artifacts automatically.
- Added separate clean trajectory and obstacle-validation views.

### Known limitations

- Validation is simulation-only; no real UAV hardware has been tested.
- Perception is map-based and simulated rather than camera-, LiDAR-, or depth-sensor-based.
- Active replanning is prototype-level and still requires deeper waypoint target-switching validation.
- Current results come from one simplified substation-style environment.
