#!/usr/bin/env python3
"""
mission_llm / llm_node.py

Role in the pipeline:  Prompt -> [LLM] -> validated mission JSON -> executor -> sim

This node NEVER talks to the robot or Nav2 directly. It only:
  1. Takes a natural-language prompt (std_msgs/String on /mission/prompt)
  2. Asks the LLM to propose a MissionPlan JSON object, grounded against
     known named zones in this specific map (zones.json)
  3. Validates that JSON against mission_schema.json + extra sanity rules
  4. Publishes the *validated* JSON on /mission/validated (std_msgs/String)
     or /mission/rejected with a reason, if it fails validation.

The LLM is explicitly kept out of the control loop: it proposes a plan,
it never issues motor/nav commands itself. If validation fails, we retry
the LLM call (bounded) rather than silently patching the JSON ourselves.
"""

import json
import os
import re
import uuid

import jsonschema
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import anthropic
    _ANTHROPIC_AVAILABLE = True
except ImportError:
    _ANTHROPIC_AVAILABLE = False

SCHEMA_PATH = os.path.join(os.path.dirname(__file__), "mission_schema.json")
ZONES_PATH = os.path.join(os.path.dirname(__file__), "zones.json")

SYSTEM_PROMPT = """You are a mission planner for a ground robot operating indoors,
in a specific known house map. You NEVER control the robot directly. You only
PROPOSE a mission plan as JSON.

Output ONLY a single JSON object. No prose, no markdown fences, no explanation.
The JSON MUST conform exactly to this schema:

{schema}

Known named zones in THIS map (map frame, metres). When the operator refers to
one of these by name or a close synonym, use its exact coordinates. If the
operator's request doesn't match any known zone, use your best judgement within
the schema's hard bounds:

{zones}

The full documented perimeter loop, in order, is: {perimeter_order}

Rules:
- "command" must be one of: patrol_loop, goto_waypoints, return_to_start
- Coordinates are in the "map" frame, in metres. Prefer known zone coordinates
  above when the prompt references a named place; otherwise stay within the
  schema's hard bounds.
- "loops" is how many times to repeat the path (1 for a one-shot trip).
- "max_speed" must never exceed 1.0 m/s; default to {default_speed} m/s if
  unspecified.
- If the instruction is ambiguous or unsafe, still produce your best-effort
  valid JSON plan within the schema bounds -- the executor and a human will
  review it; do not refuse.
"""


class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__("mission_llm_node")

        with open(SCHEMA_PATH, "r") as f:
            self.schema = json.load(f)

        self.zones = {}
        self.perimeter_order = []
        if os.path.exists(ZONES_PATH):
            with open(ZONES_PATH, "r") as f:
                zones_data = json.load(f)
            self.zones = zones_data.get("zones", {})
            self.perimeter_order = zones_data.get("perimeter_loop", [])
        else:
            self.get_logger().warn(
                f"No zones.json found at {ZONES_PATH} -- LLM will have no "
                "grounded knowledge of named rooms/zones in this map."
            )

        self.declare_parameter("model", "claude-sonnet-5")
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("default_max_speed", 0.3)

        self.model = self.get_parameter("model").value
        self.max_retries = self.get_parameter("max_retries").value
        self.default_max_speed = self.get_parameter("default_max_speed").value

        self.validated_pub = self.create_publisher(String, "/mission/validated", 10)
        self.rejected_pub = self.create_publisher(String, "/mission/rejected", 10)

        self.create_subscription(String, "/mission/prompt", self.on_prompt, 10)

        self.client = None
        if _ANTHROPIC_AVAILABLE and os.environ.get("ANTHROPIC_API_KEY"):
            self.client = anthropic.Anthropic()
        else:
            self.get_logger().warn(
                "No ANTHROPIC_API_KEY / anthropic SDK found -- "
                "falling back to a deterministic offline stub parser. "
                "Set ANTHROPIC_API_KEY to use the real LLM."
            )

        self.get_logger().info(
            f"mission_llm_node ready ({len(self.zones)} known zones loaded). "
            "Listening on /mission/prompt"
        )

    # ---------- main callback ----------

    def on_prompt(self, msg: String):
        prompt_text = msg.data.strip()
        self.get_logger().info(f"Received prompt: {prompt_text!r}")

        last_error = None
        for attempt in range(1, self.max_retries + 2):
            raw = self._call_llm(prompt_text, retry_hint=last_error)
            plan, error = self._extract_and_validate(raw)
            if plan is not None:
                self.get_logger().info(f"Validated plan on attempt {attempt}: {plan}")
                self.validated_pub.publish(String(data=json.dumps(plan)))
                return
            last_error = error
            self.get_logger().warn(f"Attempt {attempt} failed validation: {error}")

        # All retries exhausted -> reject, do NOT execute anything.
        reason = f"LLM failed to produce a schema-valid plan after retries: {last_error}"
        self.get_logger().error(reason)
        self.rejected_pub.publish(String(data=json.dumps({
            "prompt": prompt_text, "reason": reason
        })))

    # ---------- LLM call ----------

    def _zones_block(self) -> str:
        if not self.zones:
            return "(no named zones defined for this map)"
        lines = []
        for name, z in self.zones.items():
            desc = z.get("description", "")
            lines.append(f'- "{name}": x={z["x"]}, y={z["y"]}, yaw={z["yaw"]}  ({desc})')
        return "\n".join(lines)

    def _call_llm(self, prompt_text: str, retry_hint: str = None) -> str:
        if self.client is None:
            return self._offline_stub(prompt_text)

        system = SYSTEM_PROMPT.format(
            schema=json.dumps(self.schema, indent=2),
            zones=self._zones_block(),
            perimeter_order=" -> ".join(self.perimeter_order) or "(not defined)",
            default_speed=self.default_max_speed,
        )
        user_content = prompt_text
        if retry_hint:
            user_content += (
                f"\n\n[Your previous attempt was rejected: {retry_hint}. "
                "Fix it and output ONLY the corrected JSON.]"
            )

        try:
            response = self.client.messages.create(
                model=self.model,
                max_tokens=800,
                system=system,
                messages=[{"role": "user", "content": user_content}],
            )
            return "".join(
                block.text for block in response.content if block.type == "text"
            )
        except anthropic.APIError as e:
            self.get_logger().warn(
                f"Anthropic API call failed ({e}); falling back to offline stub"
            )
            return self._offline_stub(prompt_text)

    def _parse_loops(self, prompt_text: str) -> int:
        """Extract loop count from prompt text. Bounded to schema limits [1, 10]."""
        m = re.search(r"(\d+)\s*times", prompt_text, re.IGNORECASE)
        if m:
            return max(1, min(10, int(m.group(1))))
        if re.search(r"\btwice\b", prompt_text, re.IGNORECASE):
            return 2
        if re.search(r"\bonce\b", prompt_text, re.IGNORECASE):
            return 1
        return 1

    def _offline_stub(self, prompt_text: str) -> str:
        """Deterministic fallback so the pipeline is testable without an API key.
        Recognizes 'perimeter'/'patrol' to use the real documented perimeter loop
        from zones.json; a named zone mention to do a one-shot goto; otherwise a
        small default loop. This is a stub, NOT a substitute for the real LLM --
        swap ANTHROPIC_API_KEY in to exercise the actual language-understanding
        path."""
        loops = self._parse_loops(prompt_text)
        lower = prompt_text.lower()

        # Try to match a named zone mentioned directly in the prompt.
        matched_zone = None
        for name in self.zones:
            if name.replace("_", " ") in lower or name in lower:
                matched_zone = name
                break

        if ("perimeter" in lower or "patrol" in lower) and self.perimeter_order:
            waypoints = [
                {"x": self.zones[z]["x"], "y": self.zones[z]["y"], "yaw": self.zones[z]["yaw"]}
                for z in self.perimeter_order
                if z in self.zones
            ]
            command = "patrol_loop"
            notes = f"offline-stub: perimeter loop from zones.json for prompt: {prompt_text[:60]}"
        elif matched_zone:
            z = self.zones[matched_zone]
            waypoints = [{"x": z["x"], "y": z["y"], "yaw": z["yaw"]}]
            command = "goto_waypoints"
            loops = 1
            notes = f"offline-stub: matched zone '{matched_zone}' for prompt: {prompt_text[:60]}"
        else:
            # Small default square, unrelated to any real room -- last resort only.
            waypoints = [
                {"x": 1.0, "y": 0.0, "yaw": 0.0},
                {"x": 1.0, "y": 1.0, "yaw": 1.57},
                {"x": 0.0, "y": 1.0, "yaw": 3.14},
                {"x": 0.0, "y": 0.0, "yaw": -1.57},
            ]
            command = "patrol_loop"
            notes = f"offline-stub: no zone match, default loop for prompt: {prompt_text[:60]}"

        plan = {
            "mission_id": str(uuid.uuid4())[:8],
            "command": command,
            "waypoints": waypoints,
            "loops": loops,
            "max_speed": self.default_max_speed,
            "frame_id": "map",
            "notes": notes,
        }
        return json.dumps(plan)

    # ---------- validation (the guardrail layer) ----------

    def _extract_and_validate(self, raw: str):
        raw = raw.strip()
        # Strip accidental leading/trailing markdown fences if the LLM added
        # them anyway, despite instructions not to.
        raw = re.sub(r"^```(?:json)?\s*", "", raw)
        raw = re.sub(r"\s*```$", "", raw)
        raw = raw.strip()

        try:
            plan = json.loads(raw)
        except json.JSONDecodeError as e:
            return None, f"not valid JSON: {e}"

        try:
            jsonschema.validate(instance=plan, schema=self.schema)
        except jsonschema.ValidationError as e:
            return None, f"schema violation: {e.message}"

        # Extra sanity rules beyond what JSON Schema can express cleanly.
        if plan["max_speed"] > 1.0:
            return None, "max_speed exceeds hard safety cap of 1.0 m/s"
        if len(plan["waypoints"]) == 0:
            return None, "no waypoints"

        return plan, None


def main():
    rclpy.init()
    node = LLMPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()