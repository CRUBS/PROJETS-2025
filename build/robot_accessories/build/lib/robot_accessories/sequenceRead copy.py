###################################################
###												###
###			Noeud de lecture de sequence		###
###				CRUBS - Lorient					###
###	  		club.robotique.ubs@gmail.com		###
###	   			file creation 07/02/2024		###
###			last modification 07/02/2024		###
###												###
###################################################

## Import
import rclpy
from rclpy.node import Node
import time
import math

from std_msgs.msg import Int16MultiArray, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class sequenceReader(Node):
	def __init__(self):
		super().__init__('sequenceReaderNode')

		# ROS2 publishers
		self.publisherCmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
		self.publisherServos = self.create_publisher(Int16MultiArray, '/servos_cmd', 10)
		self.publisherSolar = self.create_publisher(Bool, '/solar_wheel_state', 10)

		# ROS2 subscriber
		self.subscriberLidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
		self.subscriberStart = self.create_subscription(Bool, '/starterRemoved', self.start_callback, 10)
		self.subscriberTeam = self.create_subscription(Bool, '/team', self.team_callback, 10)
		self.subscriberTOF = self.create_subscription(Int16MultiArray, '/tof_sensors', self.tof_callback, 10)

		# Timer init
		self.timerTimer = self.create_timer(0.1, self.timer_timer_callback)

		# File containing the sequence
		self.sequenceFileBleu = '/home/crubs/Documents/dev_ws/src/robot_accessories/robot_accessories/sequenceJardinBleue.txt'
		self.sequenceFileJaune = '/home/crubs/Documents/dev_ws/src/robot_accessories/robot_accessories/sequenceJardinJaune.txt'
		self.sequenceLines = []
		self.sequencePos = 0

		self.sequenceFile = '/home/crubs/Documents/dev_ws/src/robot_accessories/robot_accessories/sequenceJardinBleue.txt'

		try:
			self.sequenceData = open(self.sequenceFile, 'r')
			self.sequenceLines = self.sequenceData.readlines()
		except: print("File blue not found")

		try:
			self.sequenceDataBleu = open(self.sequenceFileBleu, 'r')
			self.sequenceLinesBleu = self.sequenceDataBleu.readlines()
		except: print("File blue not found")

		try:
			self.sequenceDataJaune = open(self.sequenceFileJaune, 'r')
			self.sequenceLinesJaune = self.sequenceDataJaune.readlines()
		except: print("File yellow not found")

		# Init variables team and starter status
		self.stop = False
		self.start = False
		self.team = "Jaune"

	def team_callback(self, msg): self.team = msg.data
	def start_callback(self, msg): self.start = msg.data
	def tof_callback(self, msg): 
		if any(data <= 20 for data in msg.data):
			self.stop = True
		else:
			self.stop = False
	
	def lidar_callback(self, msg): 
		if any(0.15 <= data <= 0.5 for data in msg.ranges if not math.isnan(data)):
			self.stop = True
		else:
			self.stop = False

	def timer_timer_callback(self):

		
		if self.team == "Bleu":
			self.sequenceLines = self.sequenceLinesBleu
		elif self.team == "Jaune":
			self.sequenceLines = self.sequenceLinesJaune 

		if self.start:
			"""
			# Read line from the sequence file
			if self.sequenceData is EOFError:
				line = [0.0,0.0,0,0,0,0]
			else:
				line  = self.sequenceData.readline().split()
			"""
			try:
				line = self.sequenceLines[self.sequencePos].split()
			except:
				self.get_logger().info("Line not found")
				time.sleep(2)
				return 0
			
			self.get_logger().info(str(self.stop))
			if not self.stop:
				self.sequencePos += 1

				vx = line[0]
				vrz = line[1]
				servosCmd = [int(line[2]), int(line[3]), int(line[4])]
				#solarWheel = bool(int(line[5]))
				if line[5] == "1":
					solarWheel = True
				else:
					solarWheel = False
			else:
				vx = 0.0
				vrz = 0.0
				servosCmd = [int(line[2]), int(line[3]), int(line[4])]
				solarWheel = False

			# Generate the message to be sent over /cmd_vel
			msgCmdVel = Twist()
			msgCmdVel.linear.x = float(vx)
			msgCmdVel.linear.y = 0.0
			msgCmdVel.linear.y = 0.0
			msgCmdVel.angular.x = 0.0
			msgCmdVel.angular.y = 0.0
			msgCmdVel.angular.z = float(vrz)

			# Generate the message to be sent over /servos_cmd
			msgServos = Int16MultiArray()
			msgServos.data = servosCmd

			# Generate the message to be sent over /solar_wheel_state
			msgSolar = Bool()
			msgSolar.data = solarWheel

			# Publishing the messages to the ros topics
			self.publisherCmdVel.publish(msgCmdVel)
			self.publisherServos.publish(msgServos)
			self.publisherSolar.publish(msgSolar)

		else :
			self.get_logger().info("Waiting to start ...")

def main(args=None):
	rclpy.init(args=args)
	node = sequenceReader()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
