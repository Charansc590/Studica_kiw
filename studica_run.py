#!/usr/bin/env python3
"""
run.py — Master sequence controller
Pick-and-place task sequence:
  1. go_to_goal(1.0, 0.0, 0.0)     → navigate to pick location
  2. wall_anchor(15, 10)            → anchor to wall precisely
  3. gripper(1, 0)                  → close gripper (pick object)
  4. go_to_goal(1.0, 0.0, -90)      → navigate toward drop location
  5. go_to_goal(1.0, -1.0, 90)      → navigate to drop location
  6. wall_anchor(15, 10)            → anchor to wall precisely
  7. gripper(0, 1)                  → open gripper (place object)
  8. move(-0.1, 0.0, 0.0)           → reverse slightly to clear object
"""

import rclpy
from studica_go_to_goal import GoToGoal
# from studica_move_dis import GoToGoal
# from wall_anchor_class import KiwiAnchorPD

# TODO: Uncomment when servo_move.py is ready
# from servo_move import Gripper

import math
import time


def run_go_to_goal(xg, yg, thg_deg):
    """Helper: create GoToGoalHack node, run it, destroy it."""
    try:
        node = GoToGoal(xg, yg, thg_deg)
        node.run()
        
        # Check if goal was reached
        if not node.done:
            print(f"⚠️  WARNING: Goal ({xg:.3f},{yg:.3f},{thg_deg:.1f}°) not reached!")
        
        node.destroy_node()
        time.sleep(0.2)  # Brief delay for cleanup
    except Exception as e:
        print(f"❌ Error in go_to_goal: {e}")
        raise



if __name__ == '__main__':
    rclpy.init()
    print("=" * 50)
    print("  Starting pick-and-place task sequence")
    print("=" * 50)

    # ── TASK 1 ── Navigate to pick location
    print("\n=== Task 1: ===")
    run_go_to_goal(1.0, 0.0, 90.0)   #wp1

    # run_go_to_goal(0.750,0.0,180.0)   #wp2

    run_go_to_goal(1.0,0.0,270.0)   #wp3
 

    run_go_to_goal(0.0,0.0,270.0)   #wp4

    run_go_to_goal(0.0,0.0,180.0)

    # run_go_to_goal(3.6,-0.3,90.0)    #wp5
    
    # run_go_to_goal(3.5,-0.75,90.0)   #wp6

    # run_go_to_goal(1.75,-1.65,180.0) #wp7

    # run_go_to_goal(1.5,-1.65,180.0)
  
    # run_go_to_goal(3.5,-0.75,90.0)
    # run_go_to_goal(3.5,-0.3,90.0)
    # run_go_to_goal(3.5,-0.75,90.0)
 # # ── TASK 2 ── Anchor to wall at pick location
    # print("\n=== Task 2: Wall anchor at pick location (front=15cm, side=10cm) ===")
    # run_wall_anchor(15, 10)

    # # ── TASK 3 ── Close gripper to pick object
    # print("\n=== Task 3: Close gripper (PICK) ===")
    # run_gripper(1, 0)

    # # ── TASK 4 ── Navigate toward drop location (first leg)
    # print("\n=== Task 4: Go to intermediate point (1.0, 0.0, -90°) ===")
    # run_go_to_goal(0.0, 0.0, 0.0)

    # # ── TASK 5 ── Navigate to drop location (second leg)
    # print("\n=== Task 5: Go to drop location (1.0, -1.0, 90°) ===")
    # run_go_to_goal(1.0, -1.0, 90.0)

    # # ── TASK 6 ── Anchor to wall at drop location
    # print("\n=== Task 6: Wall anchor at drop location (front=15cm, side=10cm) ===")
    # run_wall_anchor(15, 10)

    # # ── TASK 7 ── Open gripper to place object
    # print("\n=== Task 7: Open gripper (PLACE) ===")
    # run_gripper(0, 1)

    # # ── TASK 8 ── Reverse slightly to clear the placed object
    # print("\n=== Task 8: Reverse 10cm to clear object ===")
    # run_move(-0.1, 0.0, 0.0)

    rclpy.shutdown()
    print("\n" + "=" * 50)
    print("  ✅ All tasks completed!")
    print("=" * 50)
