# Setup and system requirements

## System requirements

- Linux with Docker installed
- X11 display (any standard Linux desktop)
- ~8 GB RAM recommended (Gazebo + Nav2)
- Port 5000 free (or remap at `docker run` time, see below)
- `ANTHROPIC_API_KEY` — **optional.** The pipeline is fully functional
  without it, via a deterministic offline planner (see "What to expect").

## Quick start (Docker — recommended)

```bash
git clone https://github.com/Pavan-Kumar-Peddireddy/Omokai-mission-pipeline.git
cd Omokai-mission-pipeline

docker build -t omokai-mission .
```

One-time, before your first run — lets the container draw to your screen:
```bash
xhost +local:docker
```

Run it:
```bash
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

Open that URL, type an instruction (e.g. *"go to room 5"* or *"patrol the
snug"*), click Send. Gazebo and RViz windows appear on your desktop via
X11; status updates stream live on the same page.

`ANTHROPIC_API_KEY` can be omitted entirely — `mission_llm` automatically
falls back to a deterministic offline planner, still grounded against the
same `zones.json`, so the full pipeline (UI, validation, executor, Nav2) is
exercisable with zero external dependency.

Per-node logs land in `logs/*.log` inside the container if anything needs
debugging; the foreground terminal only shows startup/readiness status.

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

## Troubleshooting

**Gazebo window never appears:**
Confirm you ran `xhost +local:docker` once before your first `docker run`,
and that both `-e DISPLAY=$DISPLAY` and
`-v /tmp/.X11-unix:/tmp/.X11-unix` are present in your run command exactly
as shown above.

**`docker: command not found`:**
Docker isn't installed. On Ubuntu:
```bash
sudo apt install docker.io -y
sudo usermod -aG docker $USER
newgrp docker    # or log out and back in
```

**Port 5000 already in use on your machine:**
Remap it:
```bash
docker run -it --rm -p 8080:5000 ... omokai-mission
```
then open `http://localhost:8080` instead.

**Container name conflict on re-run:**
```bash
docker stop omokai-mission 2>/dev/null; docker rm omokai-mission 2>/dev/null
```

**RViz appears with no map / blank display:**
Should not occur with the config shipped in this repo (fixed and verified
— see `TESTING.md`). If it does, confirm you're running the image built
from this repo's current `Dockerfile`, not a stale cached build:
```bash
docker build --no-cache -t omokai-mission .
```
