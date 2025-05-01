#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.cmd_vel_pub=self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.pose_subscriber=self.create_subscription(
            Pose, "/turtle1/pose",self.pose_callback,10
        )
        self.get_logger().info("Turtle controller has been started")
        self.state = 0 

    def pose_callback(self, pose: Pose):
        cmd = Twist()

        if self.state == 0:  # right
            if pose.x < 9.0:
                cmd.linear.x = 2.0
                cmd.angular.z = 0.0
            else:
                self.state = 1 
                if pose.theta!=1.57:
                    cmd.angular.z = 1

        elif self.state == 1:  # up 
            if pose.y < 9.0:
                cmd.linear.y = 2.0
            else:
                self.state = 2 

        elif self.state == 2:  # left
            if pose.x > 2.0:
                cmd.linear.x = -2.0
            else:
                self.state = 3  

        elif self.state == 3:  # down
            if pose.y > 2.0:
                cmd.linear.y = -2.0
            else:
                self.state = 0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node=TurtleControllerNode()
    rclpy.spin(node)

    rclpy.shutdown()