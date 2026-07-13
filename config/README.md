# Configuration

`substation_obstacles.json` defines the grid map used by the A* planner, the
simulated perception detector, and local replan experiments.

Important conventions:

- Grid `x` maps to local east.
- Grid `y` maps to local north.
- Cell centers convert to local NED waypoints as `(x + 0.5, y + 0.5)`.
- Raw obstacle rectangles represent physical footprints.
- Height-aware filtering decides which raw obstacles block a given flight altitude.
- Horizontal inflation expands blocking cells into planning keepout cells.

JSON does not support comments, so durable explanation belongs here and in
`docs/EXPERIMENT_PROTOCOL.md`.
