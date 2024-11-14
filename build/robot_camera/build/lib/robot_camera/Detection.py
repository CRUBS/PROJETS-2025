#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2 as cv
from cv2 import aruco
import numpy as np

class Detection:
	def __init__(self, calib_data_path, marker_size):
		self.calib_data = np.load(calib_data_path)
		self.cam_mat = self.calib_data["camMatrix"]
		self.dist_coef = self.calib_data["distCoef"]
		self.MARKER_SIZE = marker_size
		
		self.marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
		self.param_markers = aruco.DetectorParameters_create()
		
	def detectMarkers(self, frame):
		gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
		marker_corners, marker_IDs, reject = aruco.detectMarkers(
			gray_frame, self.marker_dict, parameters=self.param_markers
		)
		return marker_corners, marker_IDs

	def estimateDistance(self, marker_corners, marker_IDs):
		if marker_corners:
			rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
				marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
			)
			total_markers = range(0, marker_IDs.size)
			marker_distances = []
			marker_angles = []
			
			for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
				distance = round(np.sqrt(tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2), 2)
				angle = round(np.degrees(np.arctan2(tVec[i][0][0], tVec[i][0][2])),2)
				marker_distances.append(distance)
				marker_angles.append(angle)
			return marker_IDs, marker_distances, marker_angles
		return None, None, None

	def drawMarkerInfo(self, frame, marker_corners, marker_IDs, marker_distances):
		if marker_corners:
			for ids, corners, distances in zip(marker_IDs, marker_corners, marker_distances):
				cv.polylines(
					frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
				)
				corners = corners.reshape(4, 2)
				corners = corners.astype(int)
				top_right = tuple(corners[0].ravel())
				top_left = tuple(corners[1].ravel())
				bottom_right = tuple(corners[2].ravel())
				bottom_left = tuple(corners[3].ravel())
				
				cv.putText(
					frame,
					f"id: {ids[0]} Dist: {round(distances, 2)}",
					top_right,
					cv.FONT_HERSHEY_PLAIN,
					1.3,
					(0, 0, 255),
					2,
					cv.LINE_AA,
				)
