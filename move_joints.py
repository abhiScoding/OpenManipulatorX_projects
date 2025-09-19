#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import sys
import time


class ArmAndGripperCommander(Node):
    def __init__(self, waypoints):
        super().__init__('arm_and_gripper_commander')

        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        self.waypoints = waypoints
        self.current_index = 0

        # Run execution after node starts
        self.create_timer(1.0, self.execute_next)

    def execute_next(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("✅ Finished all waypoints.")
            return

        joints, gripper = self.waypoints[self.current_index]
        self.get_logger().info(f"➡️ Executing waypoint {self.current_index+1}/{len(self.waypoints)}")

        # --- Arm Trajectory ---
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 2
        traj_msg.points.append(point)

        self.arm_pub.publish(traj_msg)
        self.get_logger().info(f"Published arm joints: {joints}")

        # --- Gripper ---
        self.send_gripper_goal(gripper)

        # Schedule next waypoint after 3 sec
        self.current_index += 1
        self.create_timer(3.0, self.execute_next)

    def send_gripper_goal(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available!")
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = 1.0

        self.get_logger().info(f"Sending gripper goal: {position} m")
        self.gripper_client.send_goal_async(goal_msg)


def parse_waypoints(argv):
    """
    Example input:
      ros2 run my_package arm_and_gripper_commander.py \
      "0.5 0.0 -0.5 0.3 0.015,  0.0 0.5 -0.3 0.2 0.0"
    """
    if len(argv) != 2:
        print("Usage: ros2 run <your_package> arm_and_gripper_commander.py "
              "\"j1 j2 j3 j4 gripper, j1 j2 j3 j4 gripper, ...\"")
        sys.exit(1)

    raw_points = argv[1].split(',')
    waypoints = []
    for p in raw_points:
        values = [float(x) for x in p.strip().split()]
        if len(values) != 5:
            raise ValueError("Each waypoint must have 5 values: 4 joints + 1 gripper")
        joints = values[:4]
        gripper = values[4]
        waypoints.append((joints, gripper))
    return waypoints


def main(args=None):
    rclpy.init(args=args)

    waypoints = parse_waypoints(sys.argv)
    node = ArmAndGripperCommander(waypoints)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
