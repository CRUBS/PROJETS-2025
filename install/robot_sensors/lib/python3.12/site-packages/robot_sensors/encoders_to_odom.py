###################################################
###						###
###	Noeud de preparation de l odometrie	###
###		CRUBS - Lorient			###
###	  club.robotique.ubs@gmail.com		###
###	   file creation 29/01/2024		###
###	last modification 29/01/2024		###
###						###
###################################################

## import
import rclpy
from rclpy.node import Node
import time

from nav_msgs.msg import Odometry

class odomNode(Node):
	def __init__(self):
		super().__init__('encoder_odometry_node')
		
		# ROS2 publishers
		self.publisherOdom = self.create_publisher(Odometry, '/odom', 10)
		
		# ROS2 subscribers
		self.subscriptionOdom1 = self.create_subscription(Int16, '/motors/measure/encoder_1', callback_update_encoder_1, 10)
		self.subscriptionOdom2 = self.create_subscription(Int16, '/motors/measure/encoder_2', callback_update_encoder_2, 10)
		#self.subscriptionOdom3 = self.create_subscription(Int16, '/motors/measure/encoder_3', callback_update_encoder_3, 10)
		
		self.encoderValues = [0, 0, 0]
		
		# Timer init
		self.timerOdom = self.create_timer(0.1, self.timer_odom_callback)
	
	def timer_timer_callback(self):
		msg = Odometry()
		# TODO : @Leon ! Do some maths !
		# msg.data = 
		# Publishing the result
		self.publisherOdom.publish(msg)
	
	def callback_update_encoder_1(self,msg): self.encoderValues[0] = msg.data
	
	def callback_update_encoder_2(self,msg): self.encoderValues[1] = msg.data
	
	#def callback_update_encoder_3(self,msg): self.encoderValues[2] = msg.data


def main(args=None):
	rclpy.init(args=args)
	node = odomNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
