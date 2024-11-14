import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np
from std_msgs.msg import Int16

class MyNode(Node):

	"""
	__init__ initialises the global processes and variables
	"""
	def __init__(self):
		super().__init__('Wheel_odom')
		self.frequency = 0.05 	#Period between callbacks
		self.publisher_ = self.create_publisher(TwistWithCovarianceStamped, '/wheel_odom', 10)
		self.timer_ = self.create_timer(self.frequency, self.timer_callbacks)
		self.get_logger().info('Odometry node initialised')
		self.subscribtion_left = self.create_subscription(Int16, '/motors/measure/encoder_1', self.callback_left, 10)
		self.subscribtion_right = self.create_subscription(Int16, '/motors/measure/encoder_2', self.callback_right, 10)
		self.speed_right = 0
		self.speed_left  = 0
		self.wheel_radius = 0.03
		#self.conversion_encodeur_exit = 156*2*np.pi/(670*60)
		self.conversion_encoder_deplacement = lambda enc: ((enc*10/51)/(2*np.pi))*self.wheel_radius*2*np.pi*1.5
		self.L = 0.19  # Exemple d'empattement de 19 cm

	"""
	ned_to_enu takes a list in the NED (North, East, Down) format and switch it to the ENU (East, North, Up) format
	@param ned a list of coordinates in the NED format
	@return enu a list of coordinates in the ENU format
	"""
	def ned_to_enu(self,ned):
		enu = [0, 0, 0]
		enu[0], enu[1], enu[2] = ned[1], ned[0], -ned[2]
		return enu

	"""
	timer_callbacks takes the wheel speeds and transform it into an odom message published on the topic Wheel_odom
	"""
	def timer_callbacks(self):
		odom = TwistWithCovarianceStamped()

		odom.header.stamp = self.get_clock().now().to_msg()
		odom.header.frame_id = "odom"

		odom.twist.twist.linear.x = (self.speed_left + self.speed_right)/2

		odom.twist.twist.angular.z = (self.speed_right - self.speed_left)/self.L

		odom.twist.covariance = [
			0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.2
		]

		self.publisher_.publish(odom)

	'''
	callback_right takes the speed of the right wheel in rad/s and turns it into a speed in m/s
	'''
	def callback_right(self, msg):
		self.speed_right = self.conversion_encoder_deplacement(float(msg.data))
		#print(self.speed_right)
	'''
	callback_left takes the speed of the left wheel in rad/s and turns it into a speed in m/s
	'''
	def callback_left(self, msg):
		self.speed_left = self.conversion_encoder_deplacement(float(msg.data))
		#print(self.speed_left)

def main(args=None):
	rclpy.init(args=args)
	node = MyNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
