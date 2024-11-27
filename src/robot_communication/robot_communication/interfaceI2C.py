###################################################
###						###
###	  Interface I2C robot 2023-2024		###
###		CRUBS - Lorient			###
###	  club.robotique.ubs@gmail.com		###
###	   file creation 24/01/2024		###
###	last modification 26/01/2024		###
###						###
###################################################

# Import libs
import time
import random
import busio
import board
import sys
import RPi.GPIO as GPIO
import signal
import numpy as np

# Import libs ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int16, String, Bool
from sensor_msgs.msg import Imu, MagneticField

# Import libs I2C components
from adafruit_servokit import ServoKit
from rpi_lcd import LCD
from adafruit_vl53l0x import VL53L0X
from icm20948 import ICM20948


# Class for the I2C interface
class i2cInterface(Node):
	def __init__(self):
		super().__init__('interfaceI2C')

		# Raspberry s PINS declaration
		self.resetPicPin = 4 # Reset the motor microcontroller via GPIO4

		# I2C init
		self.i2c = busio.I2C(board.SCL, board.SDA)
		self.i2cBusy = False
		
		# I2C addresses declaration
		self.picAddresses = [0x50,0x51,0x52] # I2C addresses for the motor controller
		
		try: self.servoController = ServoKit(channels = 16)
		except: self.get_logger().info("servo controller not found")
		
		try: self.lcd = LCD(); time.sleep(0.5)
		except: self.get_logger().info("lcd screen not found")
		
		try: self.imu = ICM20948()
		except: self.get_logger().info("imu not found")
		
		# I2C addresses
		self.esp32Adress = 0x14
		self.pcfAddresses = [0x20, 0x21]
		self.vl53Adresses = [0x30, 0x31]
		self.vl53Group = []
		
		#team init
		self.team = "Jaune"
		
		#motors init
		self.motors_last_reset = time.time()
		
		# Pcf init
		try:
			self.i2c.writeto(self.pcfAddresses[0], bytes([0xff]))
			self.i2c.writeto(self.pcfAddresses[0], bytes([0x00]))
			self.i2c.writeto(self.pcfAddresses[1], bytes([0xff]))
			self.i2c.writeto(self.pcfAddresses[1], bytes([0x00]))
			time.sleep(0.5)
			#self.i2c.writeto(self.pcfAddresses[1], bytes([0x40]))
		except: self.get_logger().info("pcf 20 and/or 21 not found")
		
		#vl53 init
		"""
		try:
			self.i2c.writeto(self.pcfAddresses[0], bytes([0x01]))
			time.sleep(0.5)
			tempVl53 = VL53L0X(self.i2c)
			tempVl53.set_address(self.vl53Adresses[0])
			self.vl53Group.append(tempVl53)
			time.sleep(0.5)
		except: self.get_logger().info("vl53 30 not found")
		
		try:
			self.i2c.writeto(self.pcfAddresses[0], bytes([0x03]))
			time.sleep(0.5)
			tempVl53 = VL53L0X(self.i2c)
			tempVl53.set_address(self.vl53Adresses[1])
			self.vl53Group.append(tempVl53)
			time.sleep(0.5)
		except: self.get_logger().info("vl53 31 not found")
		"""
		# Servos init
		self.nbServos = 16 # Limit of number of servos in action (max 16)
		self.servosPos = [80,180,0,0,80,180,0,0,0,0,0,0,0,0,0,0]
		
		# LCD init
		self.screenString = ""

		# solar panel actuator  init
		self.solarWheelState = False
		
		# Declare motors speeds
		self.speedMotor1 = 0
		self.speedMotor2 = 0
		#self.speedMotor3 = 0

		# Esp32 variables Init
		self.lidarDistancesInt = []
				
		## ROS2 subscribers
		self.subscriptionTeam = self.create_subscription(String, '/team', self.callback_update_team, 10)
		self.subscriptionServos = self.create_subscription(Int16MultiArray, '/servos', self.callback_update_servos, 10)
		self.subscriptionScreen = self.create_subscription(String, '/screen', self.callback_update_screen, 10)
		self.subscriptionMotor1 = self.create_subscription(Int16, '/motors/cmd/speed_1', self.callback_update_motor_1, 10)
		self.subscriptionMotor2 = self.create_subscription(Int16, '/motors/cmd/speed_2', self.callback_update_motor_2, 10)
		#self.subscriptionMotor3 = self.create_subscription(Int16, '/motors/cmd/speed_3', self.callback_update_motor_3, 10)
		self.subscriptionLidarDistances = self.create_subscription(Int16MultiArray, '/lidar_distances_list', self.callback_update_lidar_distances, 10)
		self.subscriptionSolarWheelState = self.create_subscription(Bool, '/solar_wheel_state', self.callback_update_solar_wheel_state, 10)
				
		## ROS2 publishers
		self.publisherEncoder1 = self.create_publisher(Int16, '/motors/measure/encoder_1', 10)
		self.publisherEncoder2 = self.create_publisher(Int16, '/motors/measure/encoder_2', 10)
		#self.publisherEncoder3 = self.create_publisher(Int16, '/motors/measure/encoder_3', 10)
		self.publisherImu = self.create_publisher(Imu, '/imu/data_raw', 10)
		self.publisherMag = self.create_publisher(MagneticField, '/imu/mag', 10)
		self.publisherVl53 = self.create_publisher(Int16MultiArray, '/tof_sensors', 10)
				
		## ROS2 timers 
		self.timerScreen = self.create_timer(0.5, self.timer_screen_callback)
		self.timerServos = self.create_timer(0.2, self.timer_servos_callback)
		self.timerImu = self.create_timer(0.04, self.timer_imu_callback)
		self.timerVl53 = self.create_timer(0.25, self.timer_vl53_callback)
		self.timerMotors = self.create_timer(0.1, self.timer_motors_callback)
		#self.timerEncoders = self.create_timer(0.1, self.timer_encoders_callback)
		self.timerEsp32 = self.create_timer(0.2, self.timer_esp32_callback)
		self.timerPcf2 = self.create_timer(0.5, self.timer_pcf2_callback)
		
		self.publisherTimerStatusScreen = self.create_publisher(String, '/timers_status/screen', 10)
		self.publisherTimerStatusServos = self.create_publisher(String, '/timers_status/servos', 10)
		self.publisherTimerStatusImu = self.create_publisher(String, '/timers_status/imu', 10)
		self.publisherTimerStatusVl53 = self.create_publisher(String, '/timers_status/vl53', 10)
		self.publisherTimerStatusEncoders = self.create_publisher(String, '/timers_status/encoders', 10)
		self.publisherTimerStatusMotors = self.create_publisher(String, '/timers_status/motors', 10)
		self.publisherTimerStatusEsp32 = self.create_publisher(String, '/timers_status/esp32', 10)
		self.publisherTimerStatusPcf2 = self.create_publisher(String, '/timers_status/pcf2', 10)
		
	# ROS2 callback functions, update variables used by timers
	def callback_update_team(self, msg): self.team = msg.data
	
	def callback_update_servos(self, msg): self.servosPos = msg.data
		
	def callback_update_screen(self, msg): self.screenString = msg.data
		
	def callback_update_motor_1(self, msg): self.speedMotor1 = abs(msg.data)+128*(msg.data<0)
	
	def callback_update_motor_2(self, msg): self.speedMotor2 = abs(msg.data)+128*(msg.data<0)
	
	#def callback_update_motor_3(self, msg): self.speedMotor3 = abs(msg.data)+128*(msg.data<0)

	def callback_update_lidar_distances(self, msg): self.lidarDistancesInt = msg.data
	
	def callback_update_solar_wheel_state(self, msg): self.solarWheelState = msg.data
	
	def timer_pcf2_callback(self):
		#mise a jour du pcf2
		msgStatus = String()
		msgStatus.data = "pcf2 ok"
		while self.i2cBusy: pass
		self.i2cBusy = True
		pcf2_bin_list = [0,0,0,0,0,0,0,0] #etat pin digitaux pcf2 [8,7,6,5,4,3,2,1]
		try:
			#mise a jour de in1 et in2 de la roue
			if self.solarWheelState :
				if self.team in ["jaune", "Jaune", "JAUNE", "Yellow", "yellow", "YELLOW"]:
					pcf2_bin_list[0] = 1
					pcf2_bin_list[1] = 0
				elif self.team in ["bleu", "Bleu", "BLEU", "Blue", "blue", "BLUE"]:
					pcf2_bin_list[0] = 0
					pcf2_bin_list[1] = 1
				else:
					pcf2_bin_list[0] = 0
					pcf2_bin_list[1] = 0
			pcf2_byte = int(''.join(map(str, pcf2_bin_list)),2)
			self.i2c.writeto(self.pcfAddresses[1], bytes([pcf2_byte]))
		except:
			msgStatus.data = "error"
		self.i2cBusy = False
		self.publisherTimerStatusPcf2.publish(msgStatus)
	
	def timer_motors_callback(self):
		# Function sending motor cmd to the motor controller
		#self.get_logger().info("timer moteur running")
		msgStatus = String()
		msgStatus.data = "moteurs ok"
		while self.i2cBusy: pass
		self.i2cBusy = True
		try:
			self.i2c.writeto(self.picAddresses[1], bytes([self.speedMotor2]))
			#time.sleep(0.01)
			self.i2c.writeto(self.picAddresses[0], bytes([self.speedMotor1]))
			#time.sleep(0.01)
			#self.i2c.writeto(self.picAddresses[2], bytes([self.speedMotor3]))
		except:
			msgStatus.data = "error"
			"""
			if time.time() - self.motors_last_reset >= 1:
				try:
					self.i2c.writeto(self.pcfAddresses[1], bytes([0x00]))
					time.sleep(2)
					self.i2c.writeto(self.pcfAddresses[1], bytes([0x07]))
					self.motors_last_reset = time.time()
					print(self.motors_last_reset, "         i try to reset")
				except:
					pass
			"""

		self.i2cBusy = False
		msgE1 = Int16()
		msgE2 = Int16()
		#msgE3 = Int16()
		resultE1 = bytearray(1)
		resultE2 = bytearray(1)
		#resultE3 = bytearray(1)
		while self.i2cBusy: pass
		self.i2cBusy = True
		try:
			self.i2c.readfrom_into(self.picAddresses[0], resultE1)
			#time.sleep(0.01)
			self.i2c.readfrom_into(self.picAddresses[1], resultE2)
			#time.sleep(0.01)
			#self.i2c.readfrom_into(self.picAddresses[2], resultE3)

		except:
			msgStatus.data = "error"

		def decode_encoder(value, sens):
			# Convertir l'octet brut en entier
			raw_value = int.from_bytes(value, "big")
			
			# Extraire les 7 bits de données (bits 0 à 6)
			data_value = raw_value & 0b01111111  # Masque binaire pour les 7 bits de poids faible
			
			# Extraire le bit de direction (bit 7)
			direction = (raw_value & 0b10000000) >> 7  # Décalage pour obtenir uniquement le bit 7
			
			# Retourner la valeur signée selon la direction
			return sens*data_value if direction == 0 else -sens*data_value
			
		msgE1.data = decode_encoder(resultE1, -1)
		msgE2.data = decode_encoder(resultE2, 1)
		
		#msgE3.data = int.from_bytes(resultE3, "big")
		
		self.publisherEncoder1.publish(msgE1)
		self.publisherEncoder2.publish(msgE2)
		#self.publisherEncoder3.publish(msgE3)
		
		self.i2cBusy = False
		self.publisherTimerStatusEncoders.publish(msgStatus)
		
	def timer_screen_callback(self):
		# Function sending value to the lcd display
		msgStatus = String()
		msgStatus.data = "ecran ok"
		while self.i2cBusy: pass
		self.i2cBusy = True
		try:
			self.lcd.clear()
			self.lcd.text(str(self.screenString), 1, 'left')
		except:
			msgStatus.data = "Error"
		self.i2cBusy = False
		self.publisherTimerStatusScreen.publish(msgStatus)
	
	def timer_servos_callback(self):
		# Function sending values to the servo controller
		msgStatus = String()
		msgStatus.data = "servos ok"
		while self.i2cBusy: pass
		self.i2cBusy = True
		try:
			for s in range(self.nbServos):
				self.servoController.servo[s].angle = self.servosPos[s]
		except:
			msgStatus.data = "error"
		self.i2cBusy = False
		self.publisherTimerStatusServos.publish(msgStatus)
	
	def timer_imu_callback(self):
		# Function getting measurements from IMU
		imu_msg = Imu()
		mag_msg = MagneticField()
		msgStatus = String()

		msgStatus.data = "je suis rapide"  # Pas touche c'est rigolo
		while self.i2cBusy: pass
		self.i2cBusy = True

		try:
			ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
			mx, my, mz = self.imu.read_magnetometer_data()

			self.i2cBusy = False

			# Remplir le message IMU
			imu_msg.header.stamp = self.get_clock().now().to_msg()
			imu_msg.header.frame_id = "imu_link" 

			imu_msg.linear_acceleration.x = ax * 9.81  # en m/s^2
			imu_msg.linear_acceleration.y = ay * 9.81
			imu_msg.linear_acceleration.z = az * 9.81
			imu_msg.angular_velocity.x = np.radians(gx)  # en rad/s
			imu_msg.angular_velocity.y = np.radians(gy)
			imu_msg.angular_velocity.z = np.radians(gz)

			imu_msg.angular_velocity_covariance = [
				100., 0., 0.,
                0., 100., 0.,
                0., 0., 100.
			]

			imu_msg.linear_acceleration_covariance = [
				100., 0., 0.,
                0., 100., 0.,
                0., 0., 100.
			]
			
			""" imu_msg.angular_velocity_covariance = [
				0.01236, -1.01e-04, 1.01e-03,
                -1.01e-04, 0.01324, 9.84e-04,
                1.01e-03, 9.84e-04, 0.01429]

			imu_msg.linear_acceleration_covariance = [
				4.89e-06, -1.24e-08, 2.04e-07,
                -1.24e-08, 5.05e-06, 3.44e-08,
                2.04e-07, 3.44e-08, 4.47e-06
			] """

			# Remplir le message MagneticField
			mag_msg.header.stamp = self.get_clock().now().to_msg()
			mag_msg.header.frame_id = "imu_link" 

			mag_msg.magnetic_field.x = mx  # Magnétomètre en Tesla (attention à l'unité)
			mag_msg.magnetic_field.y = my
			mag_msg.magnetic_field.z = mz
			mag_msg.magnetic_field_covariance = [
				0.48492, 0.01697, 0.01263,
                0.01697, 0.48186, 0.02228,
                0.01263, 0.02228, 0.44869
			]

			# Publier les messages
			self.publisherImu.publish(imu_msg)
			self.publisherMag.publish(mag_msg)

		except OSError as e:
			self.get_logger().info(f"error: {str(e)}")
			msgStatus.data = "error"

		self.i2cBusy = False
		self.publisherTimerStatusImu.publish(msgStatus)

	def timer_esp32_callback(self):
		msgStatus = String()
		msgStatus.data = "esp32 ok"
		while self.i2cBusy: pass
		self.i2cBusy = True
		try:
			self.i2c.writeto(self.esp32Adress, bytes(list(self.lidarDistancesInt)))
		except:
			msgStatus.data = "error"
		self.i2cBusy = False
		self.publisherTimerStatusEsp32.publish(msgStatus)


	def timer_vl53_callback(self):
		# Function getting the values from range sensors
		msgStatus = String()
		msgStatus.data = "vl53 ok"
		msg = Int16MultiArray()
		vl53Ranges = []
		while self.i2cBusy: pass
		self.i2cBusy = True
		try:
			vl53Ranges.append(self.vl53Group[0].range)
			vl53Ranges.append(self.vl53Group[1].range)
			#vl53Ranges.append(self.vl53Group[2].range)
			#vl53Ranges.append(self.vl53Group[3].range)
		except:
			msgStatus.data = "error"
		self.i2cBusy = False
		msg.data = vl53Ranges
		self.publisherVl53.publish(msg)
		self.publisherTimerStatusVl53.publish(msgStatus)
		
def main(args=None):
	rclpy.init(args=args)
	node = i2cInterface()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
		
		
		
		
		
		
		
		
		
		
		
		
		
		
