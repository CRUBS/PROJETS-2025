###################################################
###						###
###	Noeud de preparation afficheur lcd	###
###		CRUBS - Lorient			###
###	  club.robotique.ubs@gmail.com		###
###	   file creation 25/01/2024		###
###	last modification 25/01/2024		###
###						###
###################################################

## import
import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Int16, Bool

class clock(Node):
	def __init__(self):
		super().__init__('clock')
		
		# ROS2 publishers
		self.publisherTimer = self.create_publisher(Int16, '/timer', 10)

		# ROS2 subscribers
		self.subscriberStart = self.create_subscription(Bool, '/starterRemoved', self.start_callback, 10)
		
		# Timer init
		self.timerTimer = self.create_timer(1, self.timer_timer_callback)
		
		# Timer value init
		self.timerValue = 0
		self.starterRemoved = False
	
	def timer_timer_callback(self):
		if self.starterRemoved == True:
			self.timerValue = self.timerValue + 1
			msg = Int16()
			msg.data = self.timerValue
			self.publisherTimer.publish(msg)

	
	def start_callback(self, msg): self.starterRemoved = msg.data


def main(args=None):
	rclpy.init(args=args)
	node = clock()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
