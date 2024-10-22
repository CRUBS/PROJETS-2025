###################################################
###						###
###	Noeud de process des distance lidar 	###
###         pour l'affichage rgb                ###
###		CRUBS - Lorient			###
###	  club.robotique.ubs@gmail.com		###
###	   file creation 25/01/2024		###
###	last modification 26/01/2024		###
###						###
###################################################

## import
import rclpy
from rclpy.node import Node
import time
import math

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class lidarLedsPrepareNode(Node):
	def __init__(self):
		super().__init__('lidarLedsPrepareNode')
		
		# ROS2 subscribers
		self.subscriptionLidar = self.create_subscription(LaserScan, '/scan', self.callback_process_lidar, 10)
		
		# ROS2 publishers
		self.publisherLidarDistances = self.create_publisher(Int16MultiArray, '/lidar_distances_list', 10)
		
		# ROS2 timers
		
		
		# Init
		
	def callback_process_lidar(self, msg):
		sectorAngleSize = 15 # size of one secteur, 15° for 24 leds (15*24 = 360°)
		anglePos = 0 # actual angle position in the sector
		angleIncrement = msg.angle_increment * (180/math.pi) # increment of angle
		distancesList = [] # list of average distances (24 values)
		extremValue = 999
		sectorIndex = 1
		for value in msg.ranges:

			anglePos += angleIncrement # calcul of actual angle pos in the sector

			if (0.30 < value < extremValue) and (not math.isnan(value)): extremValue = value

			if anglePos >= sectorAngleSize * sectorIndex:
				sectorIndex += 1
				extremValue *= 100
				if extremValue > 255: extremValue = 255
				elif extremValue < 30 : extremValue = 30
				
				distancesList.append(int(extremValue))
				extremValue = 999
		
		distancesList.reverse()
	
		# publish average distances
		msgDistance = Int16MultiArray()
		msgDistance.data = distancesList[0:24]
		self.publisherLidarDistances.publish(msgDistance)


def main(args=None):
	rclpy.init(args=args)
	node = lidarLedsPrepareNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
