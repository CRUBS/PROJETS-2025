import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class CmdDistPublisher(Node):

    def __init__(self):
        super().__init__('cmd_dist_publisher')  #Nom du noeud entre ''
        self.publisher_cmd_dist = self.create_publisher(Float32, 'cmd_dist', 10)            
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    # Callbacj=k du timer : on publie toutes les secondes la valeur 1.5
    def timer_callback(self):
        msg = Float32()
        msg.data = 1.5
        self.publisher_cmd_dist.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    cmd_dist_publisher = CmdDistPublisher()

    rclpy.spin(cmd_dist_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_dist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()