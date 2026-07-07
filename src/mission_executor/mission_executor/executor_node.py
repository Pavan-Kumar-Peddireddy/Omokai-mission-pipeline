#!/usr/bin/env python3
"""
mission_executor / executor_node.py

Role in the pipeline: validated mission JSON -> [deterministic executor] -> Nav2

This node is the ONLY thing that talks to the navigation stack. It:
  - Subscribes to /mission/validated (already-validated JSON from mission_llm)
  - Re-validates defensively (never trust upstream blindly, even your own node)
  - Converts waypoints into a Nav2 NavigateThroughPoses goal
  - Repeats the path `loops` times
  - Publishes status on /mission/status so you can watch progress / prove
    the same JSON always drives the same behaviour (auditability)

No LLM call happens anywhere in this file. Given the same JSON, this node
will always issue the same sequence of Nav2 goals.
"""

import json

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus
from tf_transformations import quaternion_from_euler


class MissionExecutorNode(Node):
    def __init__(self):
        super().__init__("mission_executor_node")

        self.declare_parameter("frame_id", "map")
        self.frame_id = self.get_parameter("frame_id").value

        self._nav_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self.status_pub = self.create_publisher(String, "/mission/status", 10)

        self.create_subscription(String, "/mission/validated", self.on_validated, 10)

        self._busy = False
        self.get_logger().info("mission_executor_node ready. Waiting for validated missions.")

    def on_validated(self, msg: String):
        if self._busy:
            self._publish_status("REJECTED", "executor busy with another mission")
            return

        try:
            plan = json.loads(msg.data)
        except json.JSONDecodeError:
            self._publish_status("REJECTED", "malformed JSON reached executor")
            return

        # Defensive re-check -- executor does not trust upstream blindly.
        required = {"mission_id", "command", "waypoints", "loops", "max_speed", "frame_id"}
        if not required.issubset(plan.keys()):
            self._publish_status("REJECTED", f"missing fields: {required - plan.keys()}")
            return
        if plan["frame_id"] != self.frame_id:
            self._publish_status("REJECTED", f"unexpected frame_id {plan['frame_id']}")
            return

        self._busy = True
        self._publish_status("ACCEPTED", plan["mission_id"])
        self._run_mission(plan)

    def _run_mission(self, plan: dict):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self._publish_status("FAILED", "navigate_through_poses server not available")
            self._busy = False
            return

        poses = [self._waypoint_to_pose(wp) for wp in plan["waypoints"]]
        loops = plan["loops"]

        self._loops_remaining = loops
        self._orig_loops = loops
        self._mission_id = plan["mission_id"]
        self._poses = poses
        self._send_next_loop()

    def _send_next_loop(self):
        if self._loops_remaining <= 0:
            self._publish_status("COMPLETED", self._mission_id)
            self._busy = False
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = self._poses

        self._publish_status(
            "EXECUTING",
            f"{self._mission_id} loop {self._loop_index()} of {self._total_loops()}",
        )

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._publish_status("FAILED", "Nav2 rejected the goal")
            self._busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._loops_remaining -= 1
            self._send_next_loop()
        elif status == GoalStatus.STATUS_CANCELED:
            self._publish_status("CANCELED", f"{self._mission_id} loop {self._loop_index()} canceled")
            self._busy = False
        else:
            self._publish_status(
                "FAILED",
                f"{self._mission_id} loop {self._loop_index()} nav goal failed, status={status}",
            )
            self._busy = False

    # ---------- helpers ----------

    def _waypoint_to_pose(self, wp: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp["x"])
        pose.pose.position.y = float(wp["y"])
        q = quaternion_from_euler(0, 0, float(wp["yaw"]))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def _loop_index(self):
        return self._orig_loops - self._loops_remaining + 1

    def _total_loops(self):
        return self._orig_loops

    def _publish_status(self, state: str, detail: str):
        self.status_pub.publish(String(data=json.dumps({"state": state, "detail": detail})))
        self.get_logger().info(f"[{state}] {detail}")


def main():
    rclpy.init()
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
