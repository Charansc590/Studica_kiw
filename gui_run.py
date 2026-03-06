#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║           KIWI BOT  —  Mission Executor  (RPi)                   ║
║                                                                  ║
║  TOPICS:                                                         ║
║    SUB  /kiwi/waypoints  std_msgs/String  (JSON waypoint list)   ║
║    SUB  /kiwi/cancel     std_msgs/String  (any msg = stop)       ║
║    PUB  /kiwi/status     std_msgs/String  (JSON feedback)        ║
║                                                                  ║
║  USAGE:                                                          ║
║    Place this file in the same folder as go_to_goal.py           ║
║    python3 run.py                                                ║
║                                                                  ║
║  INCOMING JSON (from GUI):                                       ║
║    {                                                             ║
║      "waypoints": [                                              ║
║        {"x": 1.0, "y": 0.5, "yaw": 0.0},                       ║
║        {"x": 2.0, "y": 1.0, "yaw": 90.0}                       ║
║      ]                                                           ║
║    }                                                             ║
║                                                                  ║
║  OUTGOING STATUS JSON (to GUI):                                  ║
║    {"state":"idle"}                                              ║
║    {"state":"running","current":1,"total":3,"msg":"..."}         ║
║    {"state":"reached","current":2,"total":3,"msg":"..."}         ║
║    {"state":"done",   "current":3,"total":3,"msg":"..."}         ║
║    {"state":"cancelled","msg":"Mission cancelled"}               ║
║    {"state":"error",  "current":2,"total":3,"msg":"..."}         ║
╚══════════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import threading
import math

# ── import your GoToGoal class ────────────────────────────────────────────────
from studica_go_to_goal import GoToGoal


class MissionExecutor(Node):

    def __init__(self):
        super().__init__('kiwi_mission_executor')

        # ── publishers ────────────────────────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/kiwi/status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── subscribers ───────────────────────────────────────────────────────
        self.create_subscription(String, '/kiwi/waypoints', self._wp_cb,     10)
        self.create_subscription(String, '/kiwi/cancel',    self._cancel_cb, 10)

        # ── state ─────────────────────────────────────────────────────────────
        self._mission_thread = None
        self._cancel_flag    = False
        self._running        = False
        self._active_nodes   = []  # Keep track of created nodes

        self.get_logger().info("MissionExecutor ready — waiting for /kiwi/waypoints")
        self._publish_status({"state": "idle", "msg": "Ready"})


    # ── incoming waypoints ─────────────────────────────────────────────────────
    def _wp_cb(self, msg: String):

        if self._running:
            self.get_logger().warn("Mission already running — ignoring new waypoints. Send /kiwi/cancel first.")
            self._publish_status({
                "state": "error",
                "msg": "Already running — cancel first"
            })
            return

        try:
            data      = json.loads(msg.data)
            waypoints = data["waypoints"]

            if not waypoints:
                self.get_logger().warn("Received empty waypoint list.")
                self._publish_status({"state": "error", "msg": "Empty waypoint list"})
                return

            self.get_logger().info(f"Received {len(waypoints)} waypoint(s) — starting mission")

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Bad waypoint JSON: {e}")
            self._publish_status({"state": "error", "msg": f"Bad JSON: {e}"})
            return

        # run mission in a background thread so this node keeps spinning
        self._cancel_flag = False
        self._mission_thread = threading.Thread(
            target=self._run_mission,
            args=(waypoints,),
            daemon=True
        )
        self._mission_thread.start()


    # ── cancel signal ─────────────────────────────────────────────────────────
    def _cancel_cb(self, msg: String):
        if self._running:
            self.get_logger().warn("Cancel received — stopping mission after current waypoint")
            self._cancel_flag = True
            self._stop_robot()
        else:
            self.get_logger().info("Cancel received but no mission running")


    # ── mission loop ──────────────────────────────────────────────────────────
    def _run_mission(self, waypoints: list):

        self._running = True
        total = len(waypoints)

        self._publish_status({
            "state":   "running",
            "current": 0,
            "total":   total,
            "msg":     f"Mission started — {total} waypoint(s)"
        })

        for idx, wp in enumerate(waypoints):
            current = idx + 1

            # ── check cancel before each waypoint ─────────────────────────────
            if self._cancel_flag:
                self.get_logger().warn(f"Mission cancelled before WP {current}/{total}")
                self._publish_status({
                    "state":   "cancelled",
                    "current": idx,
                    "total":   total,
                    "msg":     f"Cancelled before WP {current}/{total}"
                })
                self._running = False
                return

            # ── extract goal ──────────────────────────────────────────────────
            try:
                xg      = float(wp["x"])
                yg      = float(wp["y"])
                yaw_deg = float(wp.get("yaw", 0.0))
            except (KeyError, ValueError) as e:
                self.get_logger().error(f"Bad waypoint at index {idx}: {e}")
                self._publish_status({
                    "state":   "error",
                    "current": current,
                    "total":   total,
                    "msg":     f"Bad WP {current} data: {e}"
                })
                self._running = False
                return

            self.get_logger().info(
                f"▶ Moving to WP {current}/{total}  "
                f"x={xg:.3f} y={yg:.3f} yaw={yaw_deg:.1f}°"
            )

            self._publish_status({
                "state":   "running",
                "current": current,
                "total":   total,
                "msg":     f"Moving to WP {current}/{total}  ({xg:.2f},{yg:.2f},{yaw_deg:.0f}°)"
            })

            # ── create and run GoToGoal ────────────────────────────────────────
            success = self._execute_waypoint(xg, yg, yaw_deg, current, total)

            if not success:
                # GoToGoal failed (rclpy not ok or something went wrong)
                self._publish_status({
                    "state":   "error",
                    "current": current,
                    "total":   total,
                    "msg":     f"Failed at WP {current}/{total} ❌"
                })
                self._running = False
                return

            # ── reached ───────────────────────────────────────────────────────
            self.get_logger().info(f"✅ WP {current}/{total} reached")
            self._publish_status({
                "state":   "reached",
                "current": current,
                "total":   total,
                "msg":     f"WP {current}/{total} reached ✅  ({xg:.2f},{yg:.2f})"
            })

        # ── all done ──────────────────────────────────────────────────────────
        self.get_logger().info("🏁 All waypoints complete!")
        self._publish_status({
            "state":   "done",
            "current": total,
            "total":   total,
            "msg":     f"All {total} waypoint(s) complete 🏁"
        })
        self._running = False


    # ── single waypoint execution ─────────────────────────────────────────────
    def _execute_waypoint(self, xg, yg, yaw_deg, current, total) -> bool:
        """
        Creates a GoToGoal node, runs it to completion.
        Node is stored in _active_nodes for cleanup from main thread.
        Returns True on success, False if rclpy went down.
        """
        goal_node = None
        try:
            goal_node = GoToGoal(xg=xg, yg=yg, thg_deg=yaw_deg)
            self._active_nodes.append(goal_node)
            goal_node.run()          # blocks until goal reached or rclpy stops
            return True

        except Exception as e:
            self.get_logger().error(f"GoToGoal exception at WP {current}: {e}")
            return False


    # ── stop robot helper ────────────────────────────────────────────────────
    def _stop_robot(self):
        """Publish zero velocity to stop the robot immediately."""
        try:
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0
            stop_cmd.linear.z = 0.0
            stop_cmd.angular.x = 0.0
            stop_cmd.angular.y = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().info("[STOP] Published 0,0,0 → /cmd_vel")
        except Exception as e:
            self.get_logger().error(f"Failed to stop robot: {e}")


    # ── status publisher helper ────────────────────────────────────────────────
    def _publish_status(self, data: dict):
        try:
            msg      = String()
            msg.data = json.dumps(data)
            self.status_pub.publish(msg)
            self.get_logger().info(f"[STATUS] {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish status: {e}")


# ── main ──────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = MissionExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down MissionExecutor")
    finally:
        # Clean up all active GoToGoal nodes
        for active_node in node._active_nodes:
            try:
                executor.add_node(active_node)  # Ensure it's added to executor
                executor.remove_node(active_node)
                active_node.destroy_node()
            except Exception as e:
                print(f"[WARNING] Failed to destroy node: {e}")
        
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
