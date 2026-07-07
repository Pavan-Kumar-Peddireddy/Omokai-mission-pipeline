# Testing / verification checklist

Run these in order. Each section is independent — skip ahead if an earlier
section is already confirmed working.

## 1. Portability — clean build from a fresh clone

Proves the Dockerfile builds with zero dependency on anything already set
up on your dev machine.

```bash
cd ~/Desktop        # or any location outside your existing omokai_ws
git clone https://github.com/Pavan-Kumar-Peddireddy/Omokai-mission-pipeline.git
cd Omokai-mission-pipeline
docker build --no-cache -t omokai-mission .
```
**Expect:** `Successfully built ...` / `Successfully tagged omokai-mission:latest`,
with no errors from `colcon build` partway through.

## 2. Full container run — Gazebo, RViz, and the web UI

```bash
xhost +local:docker
docker run -d --rm --name omokai-mission \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -p 5000:5000 \
  omokai-mission

docker logs -f omokai-mission
```
**Expect:**
- Gazebo window appears with `diff_bot` in the house world
- RViz window appears with the map and robot model already visible (no
  manual "Add display" needed)
- Terminal reaches the `READY. Open http://localhost:5000` banner

Stop it when done:
```bash
docker stop omokai-mission
```

## 3. Offline planner path (no API key)

Confirms the deterministic fallback works end to end without any external
dependency.

```bash
# with the container from step 2 already running, and no ANTHROPIC_API_KEY set
```
In the browser at `http://localhost:5000`, try each of these and confirm
the robot visibly moves and status reaches `COMPLETED`:
- `go to room 5`
- `patrol the snug`
- `full house patrol`

**Expect:** `mission_llm` logs show `offline-stub: ...` matching the
correct zone; `mission_executor` logs show
`ACCEPTED -> EXECUTING -> COMPLETED`; robot visibly drives to/around the
correct room in Gazebo.

## 4. Real LLM path (API key set)

```bash
docker stop omokai-mission
docker run -d --rm --name omokai-mission \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -p 5000:5000 \
  -e ANTHROPIC_API_KEY=$ANTHROPIC_API_KEY \
  omokai-mission
```
In the browser, try an open-ended phrasing that doesn't literally contain a
zone name, e.g.:
- `check out the far side of the building`
- `sweep the biggest room twice`

**Expect:** `mission_llm` logs show a real Claude API call (not
`offline-stub`), and the resulting JSON still validates and executes
correctly — proves the language-understanding path, not just keyword
matching.

## 5. Rejection / guardrail path

Confirms invalid plans are actually blocked, not silently passed through.

```bash
ros2 topic pub /mission/prompt std_msgs/String \
  "data: 'go to coordinates 500, 500'" --once
ros2 topic echo /mission/rejected
```
(Run this from inside the container via `docker exec -it omokai-mission bash`,
or natively if running without Docker.)

**Expect:** a message on `/mission/rejected` with a clear reason (e.g. out
of schema bounds); nothing published to `/mission/validated`; no robot
movement; UI shows `[REJECTED]` cleanly without crashing.

## 6. Multi-loop patrol

```bash
# in the browser UI:
patrol the master suite 3 times
```
**Expect:** `mission_executor` logs show
`loop 1 of 3 -> loop 2 of 3 -> loop 3 of 3 -> COMPLETED`; robot visibly
repeats the same room's perimeter three times in Gazebo.

## 7. Nav2 goal-failure handling

Harder to trigger deliberately, but worth knowing what correct behaviour
looks like: if a Nav2 goal is aborted (e.g. robot physically stuck), the
executor should report `FAILED` on `/mission/status` — **not**
`COMPLETED`. If you can force this (e.g. temporarily block the robot's path
with an obstacle in Gazebo before sending a patrol command), confirm the
status reflects the real outcome.

## 8. Bare-metal (non-Docker) sanity check

Only needed once, to confirm the native install path in `SETUP.md` also
works — not required before every change, but worth doing at least once
before submission:

```bash
cd ~/omokai_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
ros2 launch mission_bringup mission_demo.launch.py
# separate terminals:
ros2 run mission_executor executor_node
ros2 run mission_llm llm_node
ros2 run mission_ui mission_ui_node
```
**Expect:** identical behaviour to the Docker path.

## 9. Config packaging sanity check

Confirms the `.rviz` / schema / zones packaging bugs found during
development stay fixed on future rebuilds:

```bash
wc -l ~/omokai_ws/install/mission_bringup/share/mission_bringup/config/nav2_default_view.rviz
grep -c "room_number" ~/omokai_ws/install/mission_llm/lib/python3.12/site-packages/mission_llm/zones.json
```
**Expect:** the `.rviz` line count matches the source file exactly (not
`0` or a stale count); `room_number` count returns `7`.
