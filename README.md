# Omokai Take-Home — Mission Pipeline (custom diff-drive ground robot, Nav2, Gazebo Harmonic)

## AI assistance disclosure

AI assistance (Claude) was used throughout for debugging support, boilerplate
scaffolding, and iterating on configuration. All architecture decisions,
root-cause diagnosis (e.g. the GPU/EGL rendering crash, SDF duplicate-entity
bug, and TF/topic mismatches), and final code were understood and validated
directly by me.

## Architecture

```
operator prompt (typed into a browser, or published via CLI)
        |
        v
  mission_ui  (Flask web UI, http://localhost:5000 -- no ROS2 CLI needed)
        |
        v
  /mission/prompt   (std_msgs/String)
        |
        v
+-------------------+   PROPOSES ONLY -- never drives the robot directly
|   mission_llm     |   - calls Claude, grounded against zones.json: named
|   (llm_node)      |     rooms in THIS house map (center/doorway/perimeter
|                    |     points), so "patrol the study" resolves to real
|                    |     coordinates instead of the LLM guessing numbers
|                    |   - retries on schema failure (bounded, not infinite)
+-------------------+
        |
        v
  /mission/validated   (std_msgs/String, JSON validated against a JSON Schema
                         + hard-coded sanity rules: speed cap, waypoint bounds,
                         known-command enum, loop-count cap)
        |
        v
+----------------------+  DETERMINISTIC + AUDITABLE: same JSON in -> same
|  mission_executor     |  robot behaviour out, every time
|  (executor_node)      |  - defensively re-validates (never trusts upstream)
+----------------------+  - converts waypoints -> Nav2 NavigateThroughPoses
        |                 - loops N times, checks real GoalStatus per loop,
        v                   publishes /mission/status
     Nav2 (planner / controller / behavior servers / bt_navigator)
        |
        v
  Custom diff-drive robot (diff_bot) in Gazebo Harmonic, localized via AMCL
  against a pre-built map of the turtlebot3_house world (see "Challenge
  attempted" below for why AMCL vs. live SLAM, and how to extend to the latter)
```

The LLM only ever *proposes* a plan; it has no path to the robot except
through the schema-validated JSON channel, and the executor re-checks that
JSON independently rather than trusting the LLM node's validation blindly.
`mission_ui` is a pure input/output surface in front of this pipeline — it
does not validate or execute anything itself.

## Challenge attempted: SLAM / autonomous navigation

**What's implemented:** AMCL-based localization against a pre-built map
(`maps/home.yaml` + `home.pgm`) combined with full Nav2 autonomous
navigation — global/local costmaps, DWB local planner, NavFn global planner,
recovery behaviors. The robot localizes itself against the map in real time
and Nav2 re-plans around any obstacles the lidar picks up; it is not
open-loop waypoint-following.

**What this is not (yet):** live SLAM (online mapping *while* navigating an
unknown environment). The map is pre-built rather than built on the fly.

**How I'd complete live SLAM from here:** swap `map_server` + `amcl` in
`nav2_bringup`'s `bringup_launch.py` for `slam_toolbox`'s
`online_async_launch.py`, which publishes `/map` live instead of loading a
static file. The rest of the pipeline needs zero changes — Nav2 only cares
that `/map` and `map->odom` exist, not which node produces them. This is a
launch-file swap, not an architecture change, precisely because the
guardrail-layer design (LLM proposes JSON, executor talks to Nav2
abstractly) keeps navigation-backend choice decoupled from everything above
it.

**A related piece of original work beyond the base rubric:** `zones.json`
captures named rooms in this specific house map (center point, doorway,
and — for enclosed rooms — a full wall-hugging perimeter for patrol sweeps),
captured via RViz's "Publish Point" tool. `mission_llm` is grounded against
this at the system-prompt level, so operators can say "patrol the study" or
"go to room 5" and get real coordinates back, not hallucinated ones. This is
what makes the natural-language layer actually useful on a specific map
rather than only usable for raw x/y prompts.

### Other two challenges (not implemented, approach below)

- **Multi-agent formations (Challenge 1):** namespace each robot instance
  (`/robot1/...`, `/robot2/...`), extend the mission JSON schema with a
  `formation` field and per-robot waypoint assignments, and add a thin
  coordination node that offsets each robot's Nav2 goal from a shared virtual
  leader pose. The LLM's job becomes translating squad-level intent ("sweep
  in a wedge") into that per-robot assignment list before it reaches the
  validator — the guardrail and executor layers barely change.
- **Vision AI target detection + follow (Challenge 3):** add a
  `mission_vision` node subscribing to the robot's camera topic, running a
  lightweight detector (e.g. Ultralytics YOLO) with a configurable
  `target_class` parameter. On detection: publish the cropped frame to
  `/mission/target_sighted` for the operator, and republish the target's
  estimated pose as a continuously-updated Nav2 goal through the *same*
  executor path used for `goto_waypoints` — "follow" is a mission whose goal
  source is a vision node instead of a fixed waypoint list, not a separate
  control path.

## Install & Run

### Docker (recommended — matches "runs on the examiner's machine")

```bash
cd omokai_ws
docker build -t omokai-mission .
docker run -it --rm -p 5000:5000 \
  -e ANTHROPIC_API_KEY=$ANTHROPIC_API_KEY \
  omokai-mission
```

The entrypoint script brings up Gazebo + Nav2, waits for Nav2's action
server, then starts `mission_executor`, `mission_llm`, and `mission_ui`,
printing a single `READY` line with the URL once everything is up:

```
==================================================================
  READY.  Open this in your browser:   http://localhost:5000
==================================================================
```

Open `http://localhost:5000`, type a prompt (e.g. *"patrol the study"* or
*"go to room 5"*), click Send, and watch `/mission/status` update live on
the same page.

If `ANTHROPIC_API_KEY` isn't set, `mission_llm` automatically falls back to
a deterministic offline planner (still grounded against `zones.json`) so the
full pipeline — UI, validation, executor, Nav2 — is exercisable without API
access.

Per-node logs land in `logs/*.log` inside the container if anything needs
debugging; the foreground terminal only shows startup/readiness status.

### Native ROS 2 Jazzy + Gazebo Harmonic (no Docker)

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-ros-gz \
    ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-tf-transformations
pip3 install --break-system-packages jsonschema anthropic flask

cd omokai_ws
colcon build --symlink-install
source install/setup.bash
export ANTHROPIC_API_KEY=sk-...

ros2 launch mission_bringup mission_demo.launch.py
# in separate terminals:
ros2 run mission_executor executor_node
ros2 run mission_llm llm_node
ros2 run mission_ui mission_ui_node   # confirm this matches `ros2 pkg executables mission_ui`
```

### Testing the LLM/JSON layer without Gazebo running

```bash
ros2 run mission_llm llm_node
ros2 topic pub /mission/prompt std_msgs/String "data: 'patrol the pantry'" --once
ros2 topic echo /mission/validated
```

## What to expect

1. Gazebo comes up headless with `diff_bot` in the house world.
2. Nav2 activates; AMCL localizes against the pre-built map (an initial pose
   is configured directly in `nav2_params.yaml`, so no manual RViz "2D Pose
   Estimate" click is needed).
3. Open `http://localhost:5000`, send a prompt.
4. `mission_llm` logs the LLM call, validates the JSON, publishes to
   `/mission/validated` (or `/mission/rejected` with a reason) — visible
   both in the container logs and live on the web UI.
5. `mission_executor` logs/publishes `ACCEPTED` -> `EXECUTING` (per loop,
   with real Nav2 goal-status checks) -> `COMPLETED` on `/mission/status`,
   and the robot visibly drives the requested room/patrol in Gazebo.

## Scaling to harder, real-world problems

- **Safety:** today's schema caps speed/coordinates statically. A real
  deployment needs the executor to check live costmap/obstacle state and
  geofences before each goal, plus a watchdog for localization loss or
  low battery.
- **LLM robustness:** move from "retry until schema-valid" to constrained
  decoding (JSON-schema-constrained generation or tool-use) so invalid
  output is structurally impossible rather than merely retried against.
- **Multi-agent:** the executor becomes a per-robot actor under a mission
  coordinator that decomposes squad-level LLM intent into per-robot JSON.
- **Vision:** target detection runs as its own node publishing structured
  observations; the executor treats "follow" and "patrol" as the same JSON
  contract with different goal sources.
- **Fleet ops / auditability:** persist every (prompt, validated JSON,
  execution outcome) tuple — the same JSON-in/JSON-out contract that makes
  the system explainable to a non-engineer operator, which is the point of
  keeping the LLM out of the control loop and putting a real UI in front of
  it rather than a CLI.

## Cited sources

| Source | License | What was used |
|---|---|---|
| [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | Apache-2.0 | Nav2 stack, `NavigateThroughPoses` action, `bringup_launch.py` pattern, AMCL |
| [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) | Apache-2.0 | `turtlebot3_house` world/model assets, originally built for an older Gazebo/ROS2 distro pairing — adapted (world SDF structure, resource paths) to run under ROS 2 Jazzy + Gazebo Harmonic |
| [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | LGPL-2.1 | Not currently wired in, but the intended drop-in replacement for live SLAM (see "Challenge attempted") |
| [Flask](https://github.com/pallets/flask) | BSD-3-Clause | `mission_ui`'s web server |
| Own original work | — | `diff_bot`: self-designed differential-drive robot description (primitive-geometry links only, no mesh files), including sensor/plugin config for Gazebo Harmonic |
| Own past work (Kody Technolab internship) | — | Nav2 param tuning conventions and SLAM/localization config patterns reused from prior work |
| [ROS-LLM (Auromix)](https://github.com/Auromix/ROS-LLM) | referenced only | Architecture reference for keeping the LLM out of the control loop (no code copied) |

`mission_llm`, `mission_executor`, `mission_ui`, and `zones.json` are
original work written for this task; no code was copied verbatim from the
reference repos above.