import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class callibrationForward(Node):
    """
    __init__ initialises the global processes and variables
    """
    def __init__(self):
        super().__init__('move_2sec')

        self.frequency = 0.05  # Period between callbacks

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(self.frequency, self.timer_callbacks)

        self.posx = 0
        self.yaw = 0
        self.cmd_pos = 0.

        self.callbackCount = 0
        self.nbMaxCountCallback = int(8/self.frequency)  # marche avant pendant 2 sec, puis stop

        self.get_logger().info('Callibration node initialised')

    def timer_callbacks(self):
        cmd_vel = Twist()
        
        if self.callbackCount<self.nbMaxCountCallback:
            cmd_vel.linear.x = 0.
            cmd_vel.angular.z = 1.0
        else:
            cmd_vel.linear.x = 0.
            cmd_vel.angular.z = 0.
        self.callbackCount+=1
        self.get_logger().info(f"Nombre de callback : {self.callbackCount} / vitesse : {cmd_vel.angular.z}")
        self.publisher_.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = callibrationForward()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
