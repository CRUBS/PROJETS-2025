###################################################
###						###
###	Noeud de preparation servosmoteurs	###
###		CRUBS - Lorient			###
###	  club.robotique.ubs@gmail.com		###
###	   file creation 01/02/2024		###
###	last modification 01/02/2024		###
###						###
###################################################

## import
import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Int16, Int16MultiArray
from geometry_msgs.msg import Twist

class motorsPrepareNode(Node):
	def __init__(self):
		super().__init__('motorsPrepareNode')
		
		# Definitions of the robot parameters
		self.wheelDistance = 0.15
		self.wheelDiameter = 0.06
		
		# ROS2 publishers
		self.publisherMotor1 = self.create_publisher(Int16, '/motors/cmd/speed_1', 10)
		self.publisherMotor2 = self.create_publisher(Int16, '/motors/cmd/speed_2', 10)
		#self.publisherMotor3 = self.create_publisher(Int16, '/motors/cmd/motor_3', 10)
		
		# ROS2 subscribers
		self.subscriberCmdVel = self.create_subscription(Twist, '/cmd_vel', self.callback_update_cmd_vel, 10)
		
		# Timer init
		#self.timerTimer = self.create_timer(1, self.timer_timer_callback)
		
		# Timer value init
		self.timerValue = 0
	
	def callback_update_cmd_vel(self,msg):
		cmdVelVx = msg.linear.x
		cmdVelVrz = msg.angular.z
		motor1Msg = Int16()
		motor2Msg = Int16()
		#self.get_logger().info('linear')
		#self.get_logger().info(str(cmdVelVx))
		#self.get_logger().info('angular')
		#self.get_logger().info(str(cmdVelVrz))
		#self.get_logger().info(str((cmdVelVx + self.wheelDistance/2 * cmdVelVrz)/51*(self.wheelDiameter/2*1.5*10)))
		motor1Msg.data = int(-(cmdVelVx - self.wheelDistance/2 * cmdVelVrz)*51/(self.wheelDiameter/2*1.5*10))
		motor2Msg.data = int((cmdVelVx + self.wheelDistance/2 * cmdVelVrz)*51/(self.wheelDiameter/2*1.5*10))
		self.publisherMotor1.publish(motor1Msg)
		self.publisherMotor2.publish(motor2Msg)

def main(args=None):
	rclpy.init(args=args)
	node = motorsPrepareNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
