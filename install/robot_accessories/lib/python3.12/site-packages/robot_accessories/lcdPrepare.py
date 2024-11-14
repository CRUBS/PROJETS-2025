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

from std_msgs.msg import Int16MultiArray, Int8, Int16, String
from geometry_msgs.msg import Twist

class lcdPrepareNode(Node):
	def __init__(self):
		super().__init__('lcdPrepareNode')
		
		# ROS2 subscribers
		self.subscriptionScore = self.create_subscription(Int8, '/score', self.callback_update_score, 10)
		self.subscriptionTimer = self.create_subscription(Int16, '/timer', self.callback_update_timer, 10)
		self.subscriptionCmdVel = self.create_subscription(Twist, '/cmd_vel', self.callback_update_cmd_vel, 10)
		self.subscriptionTeam = self.create_subscription(String, '/team', self.callback_update_team, 10)
		self.subscriptionPami = self.create_subscription(String, '/pami_status', self.callback_update_pami_status, 10)
		
		# ROS2 publishers
		self.publisherStringLcd = self.create_publisher(String, '/screen', 10)
		
		# ROS2 timers
		self.timerLCD = self.create_timer(1, self.timer_screen_callback)
		
		# Init
		self.timerValue = 0
		self.scoreValue = 23
		self.cmdvelValue = [0, 0, 0]
		self.team = ""
		self.pamiStatus = ""

	def timer_screen_callback(self):
		msg = String()
		line2 = ""
		line1 = f"{self.team} {self.timerValue}s  score: {self.scoreValue}"
		line1 = " "*int((20-len(line1))/2) + line1
		line1 = line1 + " "*int(20-len(line1))
		line2tmp = f"x:{self.cmdvelValue[0]} rz:{self.cmdvelValue[2]}"
		line2 = " "*int((20-len(line2tmp))/2) + line2tmp
		line2 = line2 + " "*int(20-len(line2tmp))
		line3 = f"{self.pamiStatus}\n"
		msg.data = line1 + line2 + line3
		self.publisherStringLcd.publish(msg)
	
	def callback_update_pami_status(self,msg): self.pamiStatus = msg.data

	def callback_update_team(self,msg): self.team = msg.data

	def callback_update_score(self,msg): self.scoreValue = msg.data
		
	def callback_update_timer(self,msg): self.timerValue = msg.data
	
	def callback_update_cmd_vel(self,msg): self.cmdvelValue[0] = round(msg.linear.x,1); self.cmdvelValue[1] = round(msg.linear.y,1); self.cmdvelValue[2] = round(msg.angular.z,1);

def main(args=None):
	rclpy.init(args=args)
	node = lcdPrepareNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
