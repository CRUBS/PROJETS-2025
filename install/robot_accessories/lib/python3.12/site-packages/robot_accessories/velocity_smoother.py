import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from time import sleep

class VelocitySmootherNode(Node):
    def __init__(self):
        super().__init__('velocity_smoother_node')

        self.cmdVelSubscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.smoothedCmdVelPublisher = self.create_publisher(Twist,'/smoothed_cmd_vel',10)

        self.timerSmoother = self.create_timer(0.1, self.timer_smoother_callback)

        self.smoothedCmdVel = Twist()
        self.lastCmdVel = Twist()
        self.cmdVel = Twist()

        self.acceleration = 0.2  # Default acceleration limit
        self.linearMin = -0.65  # Default minimum linear velocity
        self.linearMax = 0.65   # Default maximum linear velocity
        self.angularMin = -3.0  # Default minimum angular velocity
        self.angularMax = 3.0   # Default maximum angular velocity

        self.deltaAngular =  0.0
        self.deltaLinear = 0.0

    def cmd_vel_callback(self, msg):
        print("dbg 1")
        # Calculate change in velocity for linear and angular components
        self.cmdVel = msg

        self.deltaLinear = msg.linear.x - self.lastCmdVel.linear.x
        self.deltaAngular = msg.angular.z - self.lastCmdVel.angular.z

    def timer_smoother_callback(self):
        print("dbg 2")
        if abs(self.lastCmdVel.linear.x) <= self.cmdVel.linear.x :
            self.smoothedCmdVel.linear.x = self.lastCmdVel.linear.x + self.acceleration
        else:
            self.smoothedCmdVel.linear.x = 0.0

        self.smoothedCmdVelPublisher.publish(self.smoothedCmdVel)
        
        self.lastCmdVel = self.cmdVel


def main(args=None):
    rclpy.init(args=args)
    velocity_smoother_node = VelocitySmootherNode()
    rclpy.spin(velocity_smoother_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
