#!/usr/bin/env python3
"""
mission_llm / llm_node.py

Role in the pipeline:  Prompt -> [LLM] -> validated mission JSON -> executor -> sim

This node NEVER talks to the robot or Nav2 directly. It only:
  1. Takes a natural-language prompt (std_msgs/String on /mission/prompt)
  2. Asks the LLM to propose a MissionPlan JSON object, grounded against
     known named areas in this specific map (zones.json)
  3. Validates that JSON against mission_schema.json + extra sanity rules
  4. Publishes the *validated* JSON on /mission/validated (std_msgs/String)
     or /mission/rejected with a reason, if it fails validation.

zones.json schema (v4): {"areas": {name: {aka, description, room_number,
center, doorway, perimeter: [...]}}, "full_house_patrol_order": [names]}
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

Known named areas in THIS map (map frame, metres). Each has a "center" point
(for a quick goto), a "doorway" point, and often a "perimeter" -- an ordered
list of corner waypoints tracing that room's boundary walls, useful for a
full room patrol so the robot's LiDAR sweeps the whole room as it travels.
Rooms can be referred to by name OR by their room number (e.g. "room 5"):

{areas}

The full documented house patrol, in physical order, is: {patrol_order}
(walking each listed room's full perimeter in sequence).

Rules:
- "command" must be one of: patrol_loop, goto_waypoints, return_to_start
- If the operator names a specific room (by name or number) with language like
  "go to"/"visit", use that room's "center" point as a single waypoint
  (goto_waypoints, loops=1).
- If the operator asks to "patrol"/"scan"/"sweep" a specific room, use that
  room's full "perimeter" list as the waypoints (patrol_loop).
- If the operator asks for a "full house patrol"/"patrol everything"/"check
  the whole house", concatenate every room's perimeter (or center, for rooms
  without a perimeter) in the documented house patrol order into one
  waypoints list (patrol_loop).
- Coordinates are in the "map" frame, in metres. Prefer known area coordinates
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

        self.areas = {}
        self.patrol_order = []
        if os.path.exists(ZONES_PATH):
            with open(ZONES_PATH, "r") as f:
                zones_data = json.load(f)
            self.areas = zones_data.get("areas", {})
            self.patrol_order = zones_data.get("full_house_patrol_order", [])
        else:
            self.get_logger().warn(
                f"No zones.json found at {ZONES_PATH} -- LLM will have no "
                "grounded knowledge of named rooms/areas in this map."
            )

        # Build lookup helpers: name -> room dict, and "room N" -> name
        self._room_number_to_name = {
            str(r.get("room_number")): name for name, r in self.areas.items()
        }

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
            f"mission_llm_node ready ({len(self.areas)} known areas loaded). "
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

        reason = f"LLM failed to produce a schema-valid plan after retries: {last_error}"
        self.get_logger().error(reason)
        self.rejected_pub.publish(String(data=json.dumps({
            "prompt": prompt_text, "reason": reason
        })))

    # ---------- LLM call ----------

    def _areas_block(self) -> str:
        if not self.areas:
            return "(no named areas defined for this map)"
        lines = []
        for name, r in self.areas.items():
            has_perimeter = "yes" if r.get("perimeter") else "no (center/doorway only)"
            lines.append(
                f'- "{name}" ({r.get("aka", "")}): center=({r["center"]["x"]}, {r["center"]["y"]}), '
                f'doorway=({r["doorway"]["x"]}, {r["doorway"]["y"]}), has_perimeter={has_perimeter}'
            )
        return "\n".join(lines)

    def _call_llm(self, prompt_text: str, retry_hint: str = None) -> str:
        if self.client is None:
            return self._offline_stub(prompt_text)

        system = SYSTEM_PROMPT.format(
            schema=json.dumps(self.schema, indent=2),
            areas=self._areas_block(),
            patrol_order=" -> ".join(self.patrol_order) or "(not defined)",
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
                max_tokens=1200,
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
        m = re.search(r"(\d+)\s*times", prompt_text, re.IGNORECASE)
        if m:
            return max(1, min(10, int(m.group(1))))
        if re.search(r"\btwice\b", prompt_text, re.IGNORECASE):
            return 2
        if re.search(r"\bonce\b", prompt_text, re.IGNORECASE):
            return 1
        return 1

    def _match_area(self, lower_prompt: str):
        """Try to match a named area or 'room N' reference in the prompt."""
        # "room 5" / "room5" style reference
        m = re.search(r"room\s*#?\s*(\d+)", lower_prompt)
        if m and m.group(1) in self._room_number_to_name:
            return self._room_number_to_name[m.group(1)]
        # name or aka substring match, longest name first to avoid partial clashes
        for name in sorted(self.areas, key=len, reverse=True):
            display = name.replace("_", " ")
            aka = self.areas[name].get("aka", "").lower()
            if display in lower_prompt or (aka and aka in lower_prompt):
                return name
        return None

    def _offline_stub(self, prompt_text: str) -> str:
        """Deterministic fallback so the pipeline is testable without an API
        key. Handles: full house patrol, patrol/scan a specific room
        (perimeter walk), goto a specific room (center point). Falls back to
        a small default loop only if nothing matches."""
        loops = self._parse_loops(prompt_text)
        lower = prompt_text.lower()
        wants_patrol = bool(re.search(r"\b(patrol|scan|sweep)\b", lower))
        wants_full_house = bool(
            re.search(r"whole house|full house|entire house|every room|check the house", lower)
        )

        if wants_full_house and self.patrol_order:
            waypoints = []
            for name in self.patrol_order:
                room = self.areas.get(name, {})
                if room.get("perimeter"):
                    waypoints.extend(
                        {"x": p["x"], "y": p["y"], "yaw": p["yaw"]} for p in room["perimeter"]
                    )
                else:
                    c = room.get("center", {})
                    waypoints.append({"x": c["x"], "y": c["y"], "yaw": c.get("yaw", 0.0)})
            command = "patrol_loop"
            notes = f"offline-stub: full house patrol for prompt: {prompt_text[:60]}"

        else:
            matched = self._match_area(lower)
            if matched and wants_patrol and self.areas[matched].get("perimeter"):
                room = self.areas[matched]
                waypoints = [
                    {"x": p["x"], "y": p["y"], "yaw": p["yaw"]} for p in room["perimeter"]
                ]
                command = "patrol_loop"
                notes = f"offline-stub: perimeter patrol of '{matched}' for prompt: {prompt_text[:60]}"
            elif matched:
                c = self.areas[matched]["center"]
                waypoints = [{"x": c["x"], "y": c["y"], "yaw": c.get("yaw", 0.0)}]
                command = "goto_waypoints"
                loops = 1
                notes = f"offline-stub: goto '{matched}' for prompt: {prompt_text[:60]}"
            else:
                waypoints = [
                    {"x": 1.0, "y": 0.0, "yaw": 0.0},
                    {"x": 1.0, "y": 1.0, "yaw": 1.57},
                    {"x": 0.0, "y": 1.0, "yaw": 3.14},
                    {"x": 0.0, "y": 0.0, "yaw": -1.57},
                ]
                command = "patrol_loop"
                notes = f"offline-stub: no area match, default loop for prompt: {prompt_text[:60]}"

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