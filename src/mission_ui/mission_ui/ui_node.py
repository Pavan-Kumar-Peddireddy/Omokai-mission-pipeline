#!/usr/bin/env python3
"""
mission_ui / ui_node.py

Single-page browser UI so the examiner never has to touch a ROS2 CLI
command to interact with the pipeline.

Architecture (kept deliberately simple, one file, no JS framework):
  - rclpy node runs in a background thread, spinning normally.
  - Flask app runs on the main thread, serving one HTML page.
  - The two talk to each other through a small thread-safe shared state
    dict (protected by a lock) -- Flask writes a prompt into it when the
    user clicks Send, the ROS2 side reads it and publishes to
    /mission/prompt; ROS2 subscriptions write incoming status/validated/
    rejected messages into the same state, and the browser polls a
    lightweight JSON endpoint once a second to refresh the page.

This node does NOT validate or execute anything -- it is purely a human
input/output surface sitting in front of mission_llm and mission_executor.
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify, Response

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Mission Control</title>
  <style>
    body { font-family: sans-serif; max-width: 700px; margin: 40px auto; padding: 0 16px; }
    h1 { font-size: 1.4rem; }
    textarea { width: 100%; height: 70px; font-size: 1rem; padding: 8px; box-sizing: border-box; }
    button { margin-top: 8px; padding: 10px 20px; font-size: 1rem; cursor: pointer; }
    #status { margin-top: 24px; padding: 12px; background: #f4f4f4; border-radius: 6px; min-height: 40px; white-space: pre-wrap; }
    .state-ACCEPTED, .state-EXECUTING { color: #b8860b; }
    .state-COMPLETED { color: #2e7d32; font-weight: bold; }
    .state-FAILED, .state-REJECTED, .state-CANCELED { color: #c62828; font-weight: bold; }
    details { margin-top: 20px; }
    #rawlog { font-family: monospace; font-size: 0.8rem; white-space: pre-wrap; background: #111; color: #0f0; padding: 10px; border-radius: 6px; max-height: 200px; overflow-y: auto; }
  </style>
</head>
<body>
  <h1>Mission Control</h1>
  <p>Type an instruction for the robot in plain English, then click Send.</p>
  <textarea id="prompt" placeholder="e.g. Patrol the perimeter loop twice"></textarea><br>
  <button onclick="sendMission()">Send Mission</button>

  <div id="status">Waiting for a mission...</div>

  <details>
    <summary>Logs (for debugging only)</summary>
    <div id="rawlog"></div>
  </details>

<script>
async function sendMission() {
  const text = document.getElementById('prompt').value.trim();
  if (!text) return;
  document.getElementById('status').innerText = 'Sending prompt...';
  await fetch('/api/send', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({prompt: text})
  });
}

function friendlyLine(kind, obj) {
  if (kind === 'status') {
    return `<span class="state-${obj.state}">[${obj.state}]</span> ${obj.detail}`;
  }
  if (kind === 'rejected') {
    return `<span class="state-REJECTED">[REJECTED]</span> ${obj.reason}`;
  }
  return '';
}

async function poll() {
  try {
    const res = await fetch('/api/state');
    const data = await res.json();
    if (data.last_status) {
      document.getElementById('status').innerHTML = friendlyLine('status', data.last_status);
    } else if (data.last_rejected) {
      document.getElementById('status').innerHTML = friendlyLine('rejected', data.last_rejected);
    }
    document.getElementById('rawlog').innerText = data.raw_log.join('\\n');
  } catch (e) {
    // examiner's machine might briefly not have the server up yet -- ignore and retry
  }
}
setInterval(poll, 1000);
poll();
</script>
</body>
</html>
"""


class SharedState:
    """Thread-safe box shared between the Flask thread and the rclpy thread."""

    def __init__(self):
        self._lock = threading.Lock()
        self.pending_prompt = None
        self.last_status = None
        self.last_rejected = None
        self.raw_log = []

    def set_prompt(self, text: str):
        with self._lock:
            self.pending_prompt = text

    def pop_prompt(self):
        with self._lock:
            text, self.pending_prompt = self.pending_prompt, None
            return text

    def add_status(self, obj: dict):
        with self._lock:
            self.last_status = obj
            self._append_raw(f"[STATUS] {json.dumps(obj)}")

    def add_rejected(self, obj: dict):
        with self._lock:
            self.last_rejected = obj
            self._append_raw(f"[REJECTED] {json.dumps(obj)}")

    def _append_raw(self, line: str):
        self.raw_log.append(line)
        self.raw_log[:] = self.raw_log[-50:]  # keep it short, this is debug-only

    def snapshot(self):
        with self._lock:
            return {
                "last_status": self.last_status,
                "last_rejected": self.last_rejected,
                "raw_log": list(self.raw_log),
            }


state = SharedState()


class UIBridgeNode(Node):
    def __init__(self):
        super().__init__("mission_ui_node")
        self.prompt_pub = self.create_publisher(String, "/mission/prompt", 10)
        self.create_subscription(String, "/mission/status", self._on_status, 10)
        self.create_subscription(String, "/mission/rejected", self._on_rejected, 10)
        # Check for a browser-submitted prompt a few times a second.
        self.create_timer(0.2, self._flush_pending_prompt)
        self.get_logger().info("mission_ui_node ready. Open http://localhost:5000")

    def _flush_pending_prompt(self):
        text = state.pop_prompt()
        if text:
            self.prompt_pub.publish(String(data=text))
            self.get_logger().info(f"Published prompt from UI: {text!r}")

    def _on_status(self, msg: String):
        try:
            state.add_status(json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def _on_rejected(self, msg: String):
        try:
            state.add_rejected(json.loads(msg.data))
        except json.JSONDecodeError:
            pass


app = Flask(__name__)


@app.route("/")
def index():
    return Response(HTML_PAGE, mimetype="text/html")


@app.route("/api/send", methods=["POST"])
def api_send():
    data = request.get_json(force=True)
    prompt = (data or {}).get("prompt", "").strip()
    if prompt:
        state.set_prompt(prompt)
    return jsonify({"ok": True})


@app.route("/api/state")
def api_state():
    return jsonify(state.snapshot())


def main():
    rclpy.init()
    node = UIBridgeNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        # host=0.0.0.0 so it's reachable from outside the container if the
        # port is published; debug/reloader off since we're inside rclpy.
        app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
