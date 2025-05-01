#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

class MoveToCoordinates(Node):
    def __init__(self, target_pose: PoseStamped):
        super().__init__('move_to_coordinates')
        # Create an action client to communicate with MoveIt2’s move_group action server.
        self._action_client = ActionClient(self, MoveGroup, 'move_group')
        self.target_pose = target_pose

    def send_goal(self):
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        
        # Build a MoveGroup goal message.
        goal_msg = MoveGroup.Goal()

        # Build goal constraints from the target pose.
        constraints = Constraints()

        # --- Position Constraint ---
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.target_pose.header.frame_id
        pos_constraint.link_name = "ee_link"  # Change to your end-effector link name
        # Define a small tolerance region (a tiny box around the target point)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]  # 1 cm cube tolerance
        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(self.target_pose.pose)
        pos_constraint.weight = 1.0

        # --- Orientation Constraint ---
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.target_pose.header.frame_id
        orient_constraint.link_name = "ee_link"  # Change to your end-effector link name
        orient_constraint.orientation = self.target_pose.pose.orientation
        # Tolerances in radians (adjust as needed)
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0

        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)

        # Append the constraint to the goal.
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(
            "Sending goal to move to: (%.2f, %.2f, %.2f)" % (
                self.target_pose.pose.position.x,
                self.target_pose.pose.position.y,
                self.target_pose.pose.position.z))
        
        # Send the goal asynchronously.
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected by MoveIt2!")
            return

        self.get_logger().info("Goal accepted; waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("MoveIt2 result: %s" % str(result))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # Expect command-line arguments: x y z
    if len(sys.argv) < 4:
        print("Usage: go_to_coordinates.py x y z")
        return

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    except ValueError:
        print("Invalid coordinate values provided!")
        return

    # Build the target pose message.
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"  # Change if your robot uses a different base frame
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z
    # For orientation, here we set a default of no rotation (identity quaternion).
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0

    node = MoveToCoordinates(target_pose)
    node.send_goal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()