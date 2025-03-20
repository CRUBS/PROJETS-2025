import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class CmdVelRampPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_ramp_publisher_dev')  #Nom du noeud entre ''
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = create_timer(1, timer_callback)
        msg = Twist()
        msg.linear.x = 0
        self.publisher_cmd_vel.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        self.publisher_cmd_vel.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)
        



def main(args=None):
    rclpy.init(args=args)

    cmd_vel_ramp_publisher = CmdVelRampPublisher()

    rclpy.spin(cmd_vel_ramp_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_vel_ramp_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()