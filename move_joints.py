#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import sys

class PickAndPlaceCommander(Node):
    def __init__(self, waypoints):
        super().__init__('pick_and_place_commander')

        # Action clients
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        self.waypoints = waypoints
        self.current_index = 0

        # Start execution after node is ready
        self.get_logger().info("🚀 Starting pick-and-place sequence...")
        self.execute_next()

    def execute_next(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("✅ Finished all waypoints.")
            return  # Do NOT shutdown here; main thread will handle shutdown

        joints, gripper = self.waypoints[self.current_index]
        self.get_logger().info(f"➡️ Executing waypoint {self.current_index+1}/{len(self.waypoints)}")

        # --- Send arm goal ---
        self.send_arm_goal(joints, gripper)

    def send_arm_goal(self, joints, gripper_position):
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Arm action server not available!")
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 2
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Sending arm goal: {joints}")
        send_goal_future = self.arm_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda f: self.arm_done_callback(f, gripper_position))

    def arm_done_callback(self, future, gripper_position):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Arm goal rejected!")
            return

        self.get_logger().info("Arm goal accepted, waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda f: self.send_gripper_goal(gripper_position))

    def send_gripper_goal(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available!")
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = 1.0

        self.get_logger().info(f"Sending gripper goal: {position}")
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected!")
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.next_waypoint)

    def next_waypoint(self, future):
        self.current_index += 1
        self.execute_next()


def parse_waypoints(argv):
    if len(argv) != 2:
        print("Usage: ros2 run <package> pick_and_place_commander.py "
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
    node = PickAndPlaceCommander(waypoints)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()  # Safe shutdown from main thread


if __name__ == '__main__':
    main()
