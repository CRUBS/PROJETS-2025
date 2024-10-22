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

from std_msgs.msg import Int16MultiArray, Bool, String, Int8, Int16	
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class sequenceReader(Node):
	def __init__(self):
		super().__init__('sequenceReaderNode')

		# ROS2 publishers
		self.publisherCmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
		self.publisherServos = self.create_publisher(Int16MultiArray, '/servos_cmd', 10)
		self.publisherSolar = self.create_publisher(Bool, '/solar_wheel_state', 10)
		self.subscription = self.create_subscription(Int16,'/timer',self.clock_callback,10) 
		#self.publisherScore = self.create_publisher(Int8, '/score', 10)

		# ROS2 subscriber
		self.subscriberLidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
		self.subscriberStart = self.create_subscription(Bool, '/starterRemoved', self.start_callback, 10)
		self.subscriberTeam = self.create_subscription(String, '/team', self.team_callback, 10)
		#self.subscriberTOF = self.create_subscription(Int16MultiArray, '/tof_sensors', self.tof_callback, 10)
		#self.subscriberDir = self.create_subscription(Twist, '/cmd_vel', self.dir_callback, 10)

		# Timer init
		self.timerTimer = self.create_timer(0.1, self.timer_timer_callback)

		# File containing the sequence
		self.sequenceFileBleu = '/home/crubs/Documents/dev_ws/src/robot_accessories/robot_accessories/sequenceJardinBleue.txt'
		self.sequenceFileJaune = '/home/crubs/Documents/dev_ws/src/robot_accessories/robot_accessories/sequenceJardinJaune.txt'
		self.sequenceLines = []
		self.sequencePos = 0

		try:
			self.sequenceDataBleu = open(self.sequenceFileBleu, 'r')
			self.sequenceLinesBleu = self.sequenceDataBleu.readlines()
		except: self.get_logger().info("File blue not found")

		try:
			self.sequenceDataJaune = open(self.sequenceFileJaune, 'r')
			self.sequenceLinesJaune = self.sequenceDataJaune.readlines()
		except: self.get_logger().info("File yellow not found")

		# Init variables team and starter status
		self.stop = False
		#self.stop2 = False
		self.start = False
		self.team = "Jaune"
		self.desactivateLidar = False
		self.direction = 1
		self.clock = 0

	def team_callback(self, msg): self.team = msg.data
	def clock_callback(self,msg): self.clock = msg.data
	def start_callback(self, msg): self.start = msg.data

	""" def tof_callback(self, msg): 
		if int(msg.data[0]) < 300 or int(msg.data[1]) < 300:
			self.stop2 = True
		else:
			self.stop2 = False """
	
	def lidar_callback(self, msg): 
		#if any(0.15 <= data <= 0.5 for data in msg.ranges if not math.isnan(data)):
		
		if not(self.desactivateLidar):
			distanceIndex = 0
			#self.stop = False
			if self.direction == 1:
				rangeMin = 75
				rangeMax = -75

			elif self.direction == -1:
				rangeMin = 300
				rangeMax = 150

			for distanceIndex in range(rangeMin, rangeMax, -1):
				if 0.2 <= msg.ranges[distanceIndex] <= 0.55:
					self.stop = True

		else :
			self.stop = False
		#if distanceIndex >= len(msg.ranges) - 1 : self.stop = False
		
		""" self.stop = False
		for distanceIndex in range(75, -75, -1):
			if 0.2 <= msg.ranges[distanceIndex] <= 0.65:
				self.stop = True
			distanceIndex += 1 """
		
	def timer_timer_callback(self):

		if self.team == "Bleu":
			self.sequenceLines = self.sequenceLinesBleu
		elif self.team == "Jaune":
			self.sequenceLines = self.sequenceLinesJaune
		else:
			self.get_logger().info("No lines detected")

		#self.get_logger().info(str(self.sequenceLines))
		if self.start and self.clock < 100:
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
				return 0

			if line[6] == "1":
				self.desactivateLidar = False
			else:
				self.desactivateLidar = True
			
			if float(line[0]) >= 0:
				self.direction = 1
			else:
				self.direction = -1

			#self.get_logger().info(str(self.stop))
			#self.get_logger().info(f"lidar : {self.stop} | tof : {self.stop2}")			
			if not(self.stop):

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

				# Manoeuvre evitement -> retour en arriere dans la sequence
				if self.sequencePos > 0 and self.clock < 100:
					self.sequencePos -= 1
					vx = -float(line[0])
					vrz = -float(line[1])
					servosCmd = [int(line[2]), int(line[3]), int(line[4])]
					#solarWheel = bool(int(line[5]))
					if line[5] == "1":
						solarWheel = True
					else:
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

			self.stop = False

		else :
			self.get_logger().info("Waiting to start ...")

def main(args=None):
	rclpy.init(args=args)
	node = sequenceReader()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
