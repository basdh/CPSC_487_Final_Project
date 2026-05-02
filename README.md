# Spatial Final Project — Interactive SE(3) Robot Arm + Maze Challenge

This project is a browser-based Three.js lab that combines:

- an interactive 4-DOF robot arm sandbox (FK/IK, pointer interaction, spline interpolation, visualization tools), and
- a maze mini-game where xArm robots pursue the player.

The app runs fully in the browser via ES modules and an import map.

## Requirements

- A modern browser with WebGL support (Chrome, Edge, Firefox, Safari).
- Python (or any local static server) to serve files over HTTP.

Do **not** open `index.html` directly with `file://`; GLB and module loading require HTTP.

## Quick Start

From the project root:

```bash
python3 -m http.server 8080
```

Then open:

- `http://localhost:8080`

## Project Structure

- `index.html` — root page + UI overlays.
- `js/main.js` — main lab wiring, GUI, animation loop, mode switching.
- `js/MazeGame.js` — maze mini-game logic, player movement, arm pursuit.
- `js/mazeGenerator.js` — solvable maze generation and wall rectangle conversion.
- `js/RobotArm.js` — procedural arm + GLTF kinematic binding.
- `js/OptimizationController.js` — gradient-based IK optimizer.
- `js/SplineController.js` — interpolation modes (Linear, Hermite, Catmull-Rom, Quadratic/Cubic B-spline).
- `js/VisualizationController.js` — HUD, error lines, spline panel, gradients.
- `assets/xarm.glb` — robot model used in both lab and maze.

## Main Interface (Lab Mode)

When not in the maze mini-game, the main scene includes:

- Robot arm at origin
- Grid + world axes
- Interactive pointer target
- GUI controls (lil-gui, top-right)
- Bottom-left axis legend and status overlays

### Core Controls

- Mouse drag: orbit camera
- Scroll: zoom
- Arrow keys: move pointer target on the plane (lab mode)
- `W` / `S`: move pointer target vertically
- `Esc`: exits maze mode (if active)

### GUI Folders

## `Model / GLB`

- Toggle between GLB model and procedural arm.
- Set GLB URL and kinematic mapping fields:
  - control node names
  - end-effector hints
  - joint axes
- Reload GLB.

## `Pointer` (expanded by default)

- Pointer mode:
  - `No Reaction`
  - `Reach Pointer`
  - `Avoid Pointer`
- Pick scope (`GLB model` vs `Everything`)
- Pointer smoothing and keyboard speeds
- Pointer height limits
- Grip contact distance
- Show/hide pointer target and avoidance sphere

## `Optimization`

- Enable/disable optimization
- Learning rate and finite-difference settings
- Gradient clipping
- Output smoothing
- Objective weights:
  - target
  - smoothness
  - rest pose
  - avoidance
- Joint limits toggle

## `Spline`

- Spline mode:
  - `Linear`
  - `Hermite`
  - `Catmull-Rom`
  - `Quadratic B-Spline`
  - `Cubic B-Spline`
- Animation speed
- Curve/ghost/tangent/acceleration visualization toggles

## `Rotation`

- Rotation interpolation mode (`Euler` / `Quaternion SLERP`)
- Orientation ghosts toggle

## `Visualization`

- Show/hide world/origin axes
- Link frames
- End-effector trail
- Error line
- Objective text HUD
- Gradient arrows
- Spline side panel

## `Maze mini-game` (expanded by default)

- `Play maze challenge` button

## Maze Challenge

The maze mode swaps to a dedicated gameplay loop:

- Solvable maze generation (graph-verified start to exit)
- Player is the blue sphere
- Exit is the green marker
- xArms activate within a radius and pursue the player
- Contact with any arm body part triggers loss/reset
- Wall contact blocks movement (does not immediately lose)
- Level progression increases arm count

### Maze Controls

- Arrow keys: move player
- Scroll: zoom only (camera rotation disabled in maze mode)
- `Esc`: return to lab

### Maze Orientation

Camera is fixed so that:

- start (blue sphere) appears near the top-left
- exit (green marker) appears near the bottom-right

Arrow movement mapping is aligned to that view.

## Maze Generation Notes

`js/mazeGenerator.js` uses randomized depth-first search (recursive backtracker), then verifies connectivity using BFS before use.

Key points:

- Walls are represented per-cell as `{ n, e, s, w }`.
- Neighbor wall consistency is maintained.
- A path from `(0,0)` to `(cols-1, rows-1)` is asserted before rendering.
- Wall collision rectangles are derived from cell wall data (not random world blockers).

## Troubleshooting

- **Blank or failed model load**
  - Check `assets/xarm.glb` exists.
  - Confirm app is running on `http://localhost:...` and not `file://`.
- **WebGL unavailable**
  - Use another browser/GPU profile.
- **Model appears misplaced**
  - Main lab auto-centers xArm at origin and lifts it above the floor.
- **Maze feels too hard/easy**
  - Tune maze generation thinning in `js/mazeGenerator.js`.
  - Tune arm behavior in `js/MazeGame.js`.

## Extending the Project

Good places to add features:

- New spline families in `SplineController`
- Alternative IK objectives in `OptimizationController`
- New maze hazards/items in `MazeGame`
- Additional overlays in `VisualizationController`

