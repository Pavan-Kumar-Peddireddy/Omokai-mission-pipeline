

# Omokai Take-Home — Natural Language Mission Pipeline

A ground robot that takes a plain-English instruction, turns it into a
validated mission plan, and executes it deterministically in simulation —
built around a custom differential-drive robot, Nav2, and Gazebo Harmonic.


## What this demonstrates

```
"patrol the master suite"
        |
        v
   mission_ui (browser)  ->  mission_llm (proposes JSON)  ->  schema validation
        ->  mission_executor (deterministic Nav2 goals)  ->  Gazebo simulation
```

The LLM never controls the robot. It proposes a plan; a JSON Schema and a
set of hard safety rules gate everything that follows; the executor
re-validates independently and drives Nav2 the same way every time for the
same input. This separation is the core design constraint of the whole
system, and it's enforced structurally, not just by convention — see
"Architecture" below for exactly how.

🗺️ Navigation Map

The robot utilizes a segmented floor plan to understand spatial boundaries and specific target zones, allowing it to translate natural language entities (like "kitchen" or "snug") to physical (x, y) coordinates.

    Source File:
https://github.com/user-attachments/assets/a0da3467-f44f-41b7-bd48-91878d854697

    Technical Context: The segments map directly to the zone configuration in the mission_llm node, ensuring the robot knows exactly where each room begins and ends.




🎥 Mission Demonstration

See the pipeline in action as the robot receives instructions, validates them against the map, and executes the trajectory to the target zone.

    Watch the demo: 
https://github.com/user-attachments/assets/43f21518-0658-4efc-827f-732b2dd03fb0

    Mission Status: You can track the real-time execution states (ACCEPTED → EXECUTING → COMPLETED) via the /mission/status topic.

## System requirements

- Linux with Docker installed
- X11 display (any standard Linux desktop)
- ~8 GB RAM recommended (Gazebo + Nav2)
- Port 5000 free (or remap at `docker run` time)
- `ANTHROPIC_API_KEY` — optional. The pipeline is fully functional without
  it, via a deterministic offline planner described below.

## Quick start

```bash
git clone https://github.com/Pavan-Kumar-Peddireddy/Omokai-mission-pipeline.git
cd Omokai-mission-pipeline

docker build -t omokai-mission .

xhost +local:docker   # one-time, lets the container draw to your screen

docker run -it --rm -p 5000:5000 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e ANTHROPIC_API_KEY=$ANTHROPIC_API_KEY \
  omokai-mission
```

Wait for:
```
==================================================================
  READY.  Open this in your browser:   http://localhost:5000
==================================================================
```

Open that URL, type an instruction, click Send. Gazebo and RViz windows
appear on your desktop via X11; the robot's status streams live on the
same page you typed into.

If `ANTHROPIC_API_KEY` isn't set, `mission_llm` automatically falls back to
a deterministic offline planner — the full pipeline (UI, validation,
executor, Nav2) is still exercisable with zero external dependency, which
is the point of that fallback: the demo never silently fails just because
a key or network connection is missing.

## The house map and its named zones

The robot operates in a custom house layout (7 rooms + a connecting
corridor) captured from the running Nav2 map using RViz's "Publish Point"
tool. Each room is stored in `zones.json` as a real, ordered set of
boundary corners, a doorway point, and a center point — not a guessed
location.

| # | Room | Notes |
|---|------|-------|
| 1 | The Study | Robot spawn room; empty except a cupboard |
| 2 | The Pantry | Storage room; two cupboards |
| 3 | The Grand Foyer | Entrance hall; table at center |
| 4 | The Breezeway | Connecting corridor near the main entrance (no enclosed perimeter — pass-through only) |
| 5 | The Master Suite | Largest room; cupboard + table |
| 6 | The Powder Room | Small nested room inside the Master Suite |
| 7 | The Snug | Far corner room; tight, small table |

**Floor plan** (drawn to scale from the actual captured coordinates —
attach the rendered floor plan image alongside this section in your final
submission).

Because `mission_llm`'s system prompt is grounded against this file, an
operator can say "go to room 5," "patrol the snug," or "full house patrol,"
and get real map coordinates back — not a hallucinated guess. This is what
makes the natural-language layer actually usable on a specific map, rather
than only useful for raw x/y prompts.

**Three levels of instruction the pipeline understands:**
- **Goto a room** ("go to the master suite") -> single waypoint at that
  room's center.
- **Patrol a room** ("patrol the snug") -> the robot walks that room's full
  wall-hugging perimeter, so its LiDAR sweeps the whole room as it travels
  — not just a static point.
- **Full house patrol** ("full house patrol", "check the whole house") ->
  every room's perimeter, concatenated in physical order around the house,
  in one continuous mission.

## Architecture — how the LLM stays out of the control loop

This is the property the task explicitly asks candidates to justify, so
it's worth being precise about the actual mechanism, not just the claim:

**1. Import boundary.** `mission_llm` only imports `rclpy`, `std_msgs`,
`jsonschema`, and optionally `anthropic`. It never imports `nav2_msgs` and
never creates a Nav2 action client — there is no code path by which it
could send a navigation goal, not merely a convention against doing so.

**2. One-way data flow, no feedback.** `mission_llm` publishes to
`/mission/validated` and `/mission/rejected` and subscribes to nothing from
the executor. It never learns whether a mission succeeded or failed — so it
cannot loop, retry against robot state, or adapt based on what actually
happened physically. It proposes once per prompt and is finished.

**3. The schema is a hard gate, not a suggestion.** Every proposed plan is
checked with `jsonschema.validate()` against `mission_schema.json`:
`additionalProperties: false` (no smuggled extra fields), a fixed `command`
enum, bounded waypoint coordinates, a capped `max_speed`. A failing plan is
rejected and retried (bounded by `max_retries`), never silently patched or
passed through partially valid.

**4. Defense in depth.** `mission_executor` re-checks required fields and
`frame_id` independently on every message it receives on
`/mission/validated` — even though `mission_llm` already validated it. A bug
in the LLM node's own validation, or someone publishing directly to that
topic by hand, still doesn't get a free pass.

**5. Deterministic execution.** `mission_executor` has zero LLM calls and
zero randomness. Given the same validated JSON, it always issues the same
sequence of Nav2 `NavigateThroughPoses` goals, and it checks each goal's
real `GoalStatus` before continuing to the next loop — a failed or aborted
Nav2 goal is reported as `FAILED`, not silently marked `COMPLETED`. This is
what "auditable" means in practice: replaying the same JSON, with no prompt
involved at all, produces identical robot behaviour every time.

**6. The UI is I/O only.** `mission_ui` publishes prompts and displays
status; it never touches `/mission/validated` and has no path to the
executor or Nav2 either.

## Challenge attempted: SLAM / autonomous navigation

**What's implemented:** AMCL-based localization against a pre-built map,
combined with full Nav2 autonomous navigation — global/local costmaps, DWB
local planner, NavFn global planner, recovery behaviors. The robot
localizes itself in real time and Nav2 re-plans around obstacles the LiDAR
detects; this is not open-loop waypoint-following.

**What this is not, yet:** live SLAM — mapping *while* navigating an
unknown environment. The map here is pre-built rather than built on the
fly.

**How I'd complete live SLAM from here:** swap `map_server` + `amcl` in
`nav2_bringup`'s `bringup_launch.py` for `slam_toolbox`'s
`online_async_launch.py`, which publishes `/map` live instead of loading a
static file. Nothing else in the pipeline changes — Nav2 only needs `/map`
and a `map -> odom` transform to exist, not which node produces them. This
is a launch-file swap specifically because the architecture keeps
navigation-backend choice decoupled from the mission/LLM layers above it.

### Other two challenges — approach, not implemented

- **Multi-agent formations:** namespace each robot (`/robot1/...`,
  `/robot2/...`), extend the mission schema with a `formation` field and
  per-robot waypoint assignments, add a coordination node that offsets each
  robot's Nav2 goal from a shared virtual leader pose. The LLM's job
  becomes translating squad-level intent into that per-robot list before it
  reaches the validator — the guardrail and executor layers barely change.
- **Vision AI target detection + follow:** add a `mission_vision` node
  subscribing to the robot's camera, running a lightweight detector (e.g.
  Ultralytics YOLO) with a configurable `target_class`. On detection:
  publish the frame to `/mission/target_sighted` for the operator, and feed
  the target's estimated pose through the *same* executor path used for
  `goto_waypoints` — "follow" is a mission whose goal source is a vision
  node instead of a fixed waypoint list, not a separate control path.

## Running without Docker (native ROS 2 Jazzy + Gazebo Harmonic)

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-ros-gz \
    ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-tf-transformations
pip3 install --break-system-packages jsonschema anthropic flask

cd omokai_ws
colcon build --symlink-install
source install/setup.bash
export ANTHROPIC_API_KEY=sk-...   # optional

ros2 launch mission_bringup mission_demo.launch.py
# in three more terminals:
ros2 run mission_executor executor_node
ros2 run mission_llm llm_node
ros2 run mission_ui mission_ui_node
```

Open `http://localhost:5000` as above.

**Testing just the LLM/JSON layer, without Gazebo running:**
```bash
ros2 run mission_llm llm_node
ros2 topic pub /mission/prompt std_msgs/String "data: 'patrol the pantry'" --once
ros2 topic echo /mission/validated
```

## What to expect, step by step

1. Gazebo comes up with `diff_bot` in the house world; RViz shows the map
   and robot model immediately (preconfigured, no manual display setup
   needed).
2. Nav2 activates and AMCL localizes against the pre-built map.
3. Open `http://localhost:5000`, send a prompt.
4. `mission_llm` logs the call, validates the resulting JSON, publishes to
   `/mission/validated` (or `/mission/rejected` with a stated reason) —
   visible in both the container logs and the web UI.
5. `mission_executor` publishes `ACCEPTED -> EXECUTING -> COMPLETED` on
   `/mission/status` (checking real Nav2 goal status per loop), and the
   robot visibly drives the requested room or patrol route in Gazebo.

## Scaling to harder, real-world problems

- **Safety:** today's schema caps speed and coordinates statically. A real
  deployment needs the executor to check live costmap/obstacle state and
  geofences before each goal, plus a watchdog for localization loss or low
  battery.
- **LLM robustness:** move from "retry until schema-valid" toward
  constrained decoding (schema-constrained generation or tool-use), so
  invalid output is structurally impossible rather than merely retried
  against.
- **Multi-agent:** the executor becomes a per-robot actor under a mission
  coordinator that decomposes squad-level LLM intent into per-robot JSON.
- **Vision:** target detection runs as its own node publishing structured
  observations; the executor treats "follow" and "patrol" as the same JSON
  contract with a different goal source.
- **Fleet ops / auditability:** persist every (prompt, validated JSON,
  execution outcome) tuple. This is the same JSON-in/JSON-out contract that
  makes the system explainable to a non-engineer operator — which is the
  actual point of keeping the LLM out of the control loop and putting a
  real UI in front of it instead of a CLI.

## Cited sources

| Source | License | What was used |
|---|---|---|
| [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | Apache-2.0 | Nav2 stack, `NavigateThroughPoses` action, `bringup_launch.py` pattern, AMCL |
| [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) | Apache-2.0 | `turtlebot3_house` world/model assets, adapted to run under ROS 2 Jazzy + Gazebo Harmonic |
| [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | LGPL-2.1 | Not currently wired in; the intended drop-in replacement for live SLAM (see "Challenge attempted") |
| [Flask](https://github.com/pallets/flask) | BSD-3-Clause | `mission_ui`'s web server |
| Own original work | — | `diff_bot` (self-designed differential-drive robot description, primitive geometry only), `zones.json` (captured via RViz "Publish Point"), all four ROS 2 packages |
| Own past work | — | Nav2 parameter-tuning conventions and localization config patterns from prior internship work |
| [ROS-LLM (Auromix)](https://github.com/Auromix/ROS-LLM) | referenced only | Architecture reference for keeping the LLM out of the control loop; no code copied |

`mission_llm`, `mission_executor`, `mission_ui`, and `zones.json` are
original work written for this task.

## AI assistance disclosure

AI assistance (Claude) was used throughout for debugging support,
boilerplate scaffolding, and iterating on configuration. All architecture
decisions, root-cause diagnosis (the GPU/EGL rendering issue, an SDF
duplicate-entity bug, TF/topic mismatches, and a packaging bug where an
`.rviz` config file was silently never installed), and final code were
understood and validated directly by me.
