## import
import rclpy
from rclpy.node import Node
import time
import random
import busio
import board

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Imu

# import des composants I2C
from adafruit_servokit import ServoKit
from rpi_lcd import LCD
from adafruit_vl53l0x import VL53L0X
from icm20948 import ICM20948

i2c = busio.I2C(board.SCL, board.SDA)

#pca = ServoKit(channels = 16)
lcd = LCD(); time.sleep(0.5)
vl53_1 = VL53L0X(i2c)
#imu = ICM20948()


class testI2C(Node):
	def __init__(self):
		super().__init__('testI2C')

		self.i2c_busy = False

		self.nb_servos = 7
		self.servos_pos = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.screen_string = ""
		self.vl53_ranges = [0,0,0,0,0,0]

		self.publisher_imu = self.create_publisher(Imu, '/imu', 10)
		self.publisher_vl53 = self.create_publisher(Int16MultiArray, '/vl53', 10)

		self.subscription_screen = self.create_subscription(Int16MultiArray, '/servos', self.callback_update_servos, 10)
		self.subscription_servos = self.create_subscription(String, '/screen', self.callback_update_screen, 10)

		self.timer_screen = self.create_timer(0.5, self.timer_screen_callback)
		#self.timer_servos = self.create_timer(0.2, self.timer_servos_callback)
		#self.time_imu = self.create_timer(0.1, self.timer_imu_callback)
		self.timer_vl53 = self.create_timer(0.5, self.timer_vl53_callback)

		self.publisher_timer_status_screen = self.create_publisher(String, '/timers_status/screen', 10)
		self.publisher_timer_status_servos = self.create_publisher(String, '/timers_status/servos', 10)
		self.publisher_timer_status_imu = self.create_publisher(String, '/timers_status/imu', 10)
		self.publisher_timer_status_vl53 = self.create_publisher(String, '/timers_status/vl53', 10)

	def callback_update_servos(self, msg):
		self.servos_pos = msg.data

	def callback_update_screen(self, msg):
		self.screen_string = msg.data

	def timer_screen_callback(self):
		msgStatus = String()
		msgStatus.data = "ca marche"
		while self.i2c_busy: pass
		self.i2c_busy = True
		try:
			lcd.clear()
			lcd.text(str(self.screen_string), 1, 'center')
		except:
			msgStatus.data = "Error"
		self.i2c_busy = False
		self.publisher_timer_status_screen.publish(msgStatus)

	def timer_servos_callback(self):
		while self.i2c_busy: pass
		self.i2c_busy = True
		for s in range(self.nb_servos):
			pca.servo[s].angle = self.servos_pos[s]
		self.i2c_busy = False

	def timer_imu_callback(self):
		msg = Imu()
		while self.i2c_busy: pass
		self.i2c_busy = True
		(ax, ay, az, gx, gy, gz) = imu.read_accelerometer_gyro_data()
		self.i2c_busy = False
		msg.linear_acceleration.x = ax #en g
		msg.linear_acceleration.y = ay
		msg.linear_acceleration.z = az
		msg.angular_velocity.x = gx #en rad/s
		msg.angular_velocity.y = gy
		msg.angular_velocity.z = gz
		self.publisher_imu.publish(msg)

	def timer_vl53_callback(self):
		msg = Int16MultiArray()
		while self.i2c_busy: pass
		self.i2c_busy = True
		self.vl53_ranges[0] = vl53_1.range
		self.i2c_busy = False
		msg.data = self.vl53_ranges
		self.publisher_vl53.publish(msg)


def main(args=None):
	rclpy.init(args=args)
	node = testI2C()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
