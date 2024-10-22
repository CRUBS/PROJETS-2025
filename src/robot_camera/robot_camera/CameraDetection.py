#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import rclpy
from rclpy.node import Node

import sys
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from .Detection import Detection

class CameraDetection(Node):

	def __init__(self, calib_data_path="src/robot_camera/calib_data_v2/MultiMatrix.npz", timer_period = 0.5):
		super().__init__('camera_detection')
		self.image_sub = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
		self.image_pub = self.create_publisher(Float32MultiArray, '/camera_data', 10)

		self.detection = Detection(calib_data_path, 2)
		self.bridge = CvBridge()

	def listener_callback(self, data):

		msg = Float32MultiArray()

		try:
			frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		marker_corners, marker_IDs = self.detection.detectMarkers(frame)
		marker_IDs, marker_distances, marker_angles = self.detection.estimateDistance(marker_corners, marker_IDs)

		# camera data acquisition
		if marker_IDs is not None and marker_IDs.any():
			for i in range (len(marker_IDs)):
				msg.data.append(marker_IDs[i])
				msg.data.append(marker_distances[i])
				msg.data.append(marker_angles[i])
		else:
			msg.data = [0.0]
			self.get_logger().info("no markers")
		
		# node publication
		try:	
			self.image_pub.publish(msg)
			data_str = ', '.join(str(data) for data in msg.data)
			self.get_logger().info(f'Valeurs: {data_str}')
		except CvBridgeError as e:
			print(e)

def main(args=None):

	rclpy.init(args=args)
	camera_detection = CameraDetection()

	try:
		rclpy.spin(camera_detection)
	except KeyboardInterrupt:
		print("Shutting down")

	camera_detection.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main(sys.argv)
