#!/bin/bash
# Omokai mission pipeline entrypoint.
# Starts Gazebo+Nav2 bringup, then the three mission nodes, so the
# examiner runs ONE `docker run` command and gets the full pipeline.
# Keeps the foreground terminal clean: per-node logs go to files, only
# essential status/readiness/warning lines print to the console.
set -e

source /opt/ros/jazzy/setup.bash
source /omokai_ws/install/setup.bash

LOG_DIR=/omokai_ws/logs
mkdir -p "$LOG_DIR"

echo "=================================================================="
echo " Omokai Mission Pipeline - startup checks"
echo "=================================================================="

# --- Preflight checks: catch what we CAN detect from inside the container ---
if [ -z "$DISPLAY" ]; then
  echo "WARNING: No DISPLAY environment variable set."
  echo "  Gazebo's GUI will not be visible. Re-run with:"
  echo "    -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix"
fi

if [ ! -d /tmp/.X11-unix ]; then
  echo "WARNING: /tmp/.X11-unix not mounted -- Gazebo GUI likely will not render."
fi

TOTAL_MEM_MB=$(free -m | awk '/^Mem:/{print $2}')
if [ "$TOTAL_MEM_MB" -lt 4096 ]; then
  echo "WARNING: Only ${TOTAL_MEM_MB}MB RAM visible to this container."
  echo "  Gazebo + Nav2 typically want 4-8GB+; simulation may be slow or fail to start."
fi

if [ -z "$ANTHROPIC_API_KEY" ]; then
  echo "NOTE: ANTHROPIC_API_KEY not set -- mission_llm will use its offline"
  echo "  deterministic planner instead of the live LLM. This is expected"
  echo "  and the pipeline is fully functional either way."
fi

echo ""
echo "Starting Gazebo + Nav2 bringup (log: $LOG_DIR/bringup.log)..."
ros2 launch mission_bringup mission_demo.launch.py > "$LOG_DIR/bringup.log" 2>&1 &
BRINGUP_PID=$!

echo "Waiting for Nav2 to come online..."
NAV2_READY=0
for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q navigate_through_poses; then
    NAV2_READY=1
    break
  fi
  sleep 1
done
if [ "$NAV2_READY" -eq 0 ]; then
  echo "WARNING: Nav2 action server not detected after 60s. Check $LOG_DIR/bringup.log."
  echo "  Continuing anyway -- mission_executor will retry when a prompt is sent."
fi

echo "Starting mission_executor, mission_llm, mission_ui (logs in $LOG_DIR/)..."
ros2 run mission_executor executor_node > "$LOG_DIR/executor.log" 2>&1 &
EXECUTOR_PID=$!
ros2 run mission_llm llm_node > "$LOG_DIR/llm.log" 2>&1 &
LLM_PID=$!
ros2 run mission_ui mission_ui_node > "$LOG_DIR/ui.log" 2>&1 &
UI_PID=$!

echo "Waiting for Mission Control UI to come online..."
UI_READY=0
for i in $(seq 1 30); do
  if curl -s -o /dev/null http://localhost:5000; then
    UI_READY=1
    break
  fi
  sleep 1
done

echo ""
echo "=================================================================="
if [ "$UI_READY" -eq 1 ]; then
  echo "  READY.  Open this in your browser:   http://localhost:5000"
else
  echo "  UI did not respond after 30s -- check $LOG_DIR/ui.log"
  echo "  It may still come up shortly; try http://localhost:5000 manually."
fi
echo "  Logs for debugging only: $LOG_DIR/*.log (inside the container)"
echo "=================================================================="
echo ""

# If any one process dies, bring the whole container down rather than
# leaving a half-working pipeline running silently.
wait -n
echo "A pipeline process exited unexpectedly. Check logs in $LOG_DIR/."
kill $BRINGUP_PID $EXECUTOR_PID $LLM_PID $UI_PID 2>/dev/null || true
exit 1