import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class MyNode(Node):
    """
    __init__ initialises the global processes and variables
    """
    def __init__(self):
        super().__init__('Cmd_test')

        self.frequency = 0.05  # Period between callbacks

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odomSubscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback , 10)
        self.cmdSubscriber_ = self.create_subscription(Float32, '/cmd_pos', self.cmd_callback , 10)
        self.timer_ = self.create_timer(self.frequency, self.timer_callbacks)

        self.posx = 0
        self.yaw = 0
        self.cmd_pos = 0.

        self.get_logger().info('Cmd node initialised')

    def odom_callback(self, msg):
        self.posx = msg.pose.pose.position.x
        self.yaw = (msg.pose.pose.orientation.z)*(180/3.1415)
        self.get_logger().info(f'Yaw : {self.yaw}, X : {self.posx}')

    def cmd_callback(self, msg): self.cmd_pos = msg.data

    def timer_callbacks(self):

        cmd_vel = Twist()

        if self.cmd_pos < 5:
            if self.posx < self.cmd_pos:
                cmd_vel.linear.x = 0.5
                cmd_vel.angular.z = 0.
            else :
                cmd_vel.linear.x = 0.
                cmd_vel.angular.z = 0.
        else :
            if self.yaw < self.cmd_pos:
                cmd_vel.linear.x = 0.
                cmd_vel.angular.z = 1.
            else :
                cmd_vel.linear.x = 0.
                cmd_vel.angular.z = 0.

        self.publisher_.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
