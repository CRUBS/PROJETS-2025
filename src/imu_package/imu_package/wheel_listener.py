import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from std_msgs.msg import Int16
from math import cos, sin
import tf_transformations  # Importation pour les quaternions
import time

class MyNode(Node):
    """
    __init__ initialises the global processes and variables
    """
    def __init__(self):
        super().__init__('wheel_odom')
        self.frequency = 0.05  # Period between callbacks
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer_ = self.create_timer(self.frequency, self.timer_callbacks)
        self.get_logger().info('Odometry node initialised')

        self.subscribtion_left = self.create_subscription(Int16, '/motors/measure/encoder_1', self.callback_left, 10)
        self.subscribtion_right = self.create_subscription(Int16, '/motors/measure/encoder_2', self.callback_right, 10)

        self.wheel_radius = 0.03
        self.L = 0.15  # Wheelbase in meters (19 cm)
        
        # Initializing variables to store state
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.speed_right = 0.0
        self.speed_left = 0.0

        self.last_time = self.get_clock().now()

    def conversion_deplacement_encodeurs(self, value):
        # Ensure that 'value' is always treated as a float
        try:
            return (float(value) * 10 / 51) / (2 * np.pi) * self.wheel_radius * 2 * np.pi * 1.5
        except ValueError as e:
            self.get_logger().error(f"Conversion error: {e}")
            return 0.0

    """
    timer_callbacks takes the wheel speeds and transforms them into an odom message published on the topic Wheel_odom
    """
    def timer_callbacks(self):
        current_time = self.get_clock().now()
        duration = current_time - self.last_time
        dt = duration.nanoseconds / 1e9  # Convert nanoseconds to seconds

        # Compute linear and angular speed
        linear_speed = (self.speed_left + self.speed_right) / 2
        angular_speed = (self.speed_right - self.speed_left) / self.L

        # Update yaw, posx, posy based on the kinematics
        self.yaw += angular_speed * dt
        self.posx += linear_speed * cos(self.yaw) * dt
        self.posy += linear_speed * sin(self.yaw) * dt

        # Create quaternion for the yaw using tf_transformations
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)  # RPY (Roll, Pitch, Yaw)

        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Fill pose (ensure all positions are floats)
        odom.pose.pose.position.x = float(self.posx)
        odom.pose.pose.position.y = float(self.posy)
        odom.pose.pose.position.z = 0.0  # Ensure 0 is treated as a float
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])  # Quaternion from q

        # Fill covariance (just an example, adjust as needed)
        odom.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.5]

        # Fill twist (linear and angular velocities, ensure they are floats)
        odom.twist.twist.linear.x = float(linear_speed)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(angular_speed)

        # Publish Odometry
        self.publisher_.publish(odom)

        # Broadcast the transform (Odometry to base_footprint)
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time.to_msg()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_footprint"

        odom_trans.transform.translation.x = float(self.posx)
        odom_trans.transform.translation.y = float(self.posy)
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation.x = q[0]
        odom_trans.transform.rotation.y = q[1]
        odom_trans.transform.rotation.z = q[2]
        odom_trans.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(odom_trans)

        # Update last_time
        self.last_time = current_time

    """
    callback_right takes the speed of the right wheel in rad/s and turns it into a speed in m/s
    """
    def callback_right(self, msg):
        try:
            self.speed_right = self.conversion_deplacement_encodeurs(float(msg.data))
        except ValueError as e:
            self.get_logger().error(f"Invalid value for right wheel: {msg.data} - Error: {e}")
            self.speed_right = 0.0

    """
    callback_left takes the speed of the left wheel in rad/s and turns it into a speed in m/s
    """
    def callback_left(self, msg):
        try:
            self.speed_left = self.conversion_deplacement_encodeurs(float(msg.data))
        except ValueError as e:
            self.get_logger().error(f"Invalid value for left wheel: {msg.data} - Error: {e}")
            self.speed_left = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
