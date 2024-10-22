## import
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import time
import random
# import des composants I2C
from adafruit_servokit import ServoKit
from rpi_lcd import LCD

pca = ServoKit(channels = 16)

class interfaceI2C(Node):
	def __init__(self):
		super().__init__('interfaceI2C')
		self.pos = [[10, 165],[0, 120, 15, 40], [180, 80, 175, 150]]
		self.subscription = self.create_subscription(Int16, '/servos', self.drive_servos, 10)
		self.subscription

	def drive_servos(self, msg):
		match msg.data:
			#Ouverture pour attrapage
			case 0:
				pca.servo[0].angle = self.pos[0][0]
				pca.servo[1].angle = self.pos[1][1]
				pca.servo[2].angle = self.pos[2][1]
			#Fermeture pour attrapage des plantes
			case 1:
				pca.servo[0].angle = self.pos[0][0]
				pca.servo[1].angle = self.pos[1][2]
				pca.servo[2].angle = self.pos[2][2]
			#Fermeture pour attrapage des pots
			case 2:
				pca.servo[0].angle = self.pos[0][0]
				pca.servo[1].angle = self.pos[1][3]
				pca.servo[2].angle = self.pos[2][3]
			#Ouverture pour lachage des pots
			case 3:
				pca.servo[0].angle = self.pos[0][1]
				pca.servo[1].angle = self.pos[1][1]
				pca.servo[2].angle = self.pos[2][1]
			#Fermeture pots pour déplacements
			case 4:
				pca.servo[0].angle = self.pos[0][1]
				pca.servo[1].angle = self.pos[1][3]
				pca.servo[2].angle = self.pos[2][3]
			#Fermeture plantes pour déplacements
			case 5:
				pca.servo[0].angle = self.pos[0][1]
				pca.servo[1].angle = self.pos[1][2]
				pca.servo[2].angle = self.pos[2][2]
			#Position de repos
			case _:
				pca.servo[0].angle = self.pos[0][1]
				pca.servo[1].angle = self.pos[1][0]
				pca.servo[2].angle = self.pos[2][0]

def main(args=None):
	rclpy.init(args=args)
	node = interfaceI2C()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
