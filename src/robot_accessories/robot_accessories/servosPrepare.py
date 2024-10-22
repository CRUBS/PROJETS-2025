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

from std_msgs.msg import Int16, Int16MultiArray, Bool, String

class servoPrepareNode(Node):
	def __init__(self):
		super().__init__('servosPrepare')

		# Definitions of the poses of the servos
		# ferme plante
		# ferme pot
		# ouvert
		# defaut #55 au lieu de 70
		self.team = 'bleu'
		self.posPick = [[150,30],[130,50],[110,45],[180,0]]
		self.posElev = [10, 60, 100, 150]
		self.posSolar = [0, 130]
		# Asc Pince Pince 0 Asc Pince Pince
		self.servoCmd = [10,180,0,0,10,180,0,0,0,90,0,0,0,0,0,0,0]

		# ROS2 publishers
		self.publisherServos = self.create_publisher(Int16MultiArray, '/servos', 10)

		# ROS2 Subscribers
		self.subscriptionServos = self.create_subscription(Int16MultiArray, '/servos_cmd', self.callback_update_servos, 10)
		self.subscriptionSolarWheelState = self.create_subscription(Bool, '/solar_wheel_state', self.callback_update_solar, 10)
		self.subscriptionTeam = self.create_subscription(String, '/team', self.callback_update_team, 10)

		# Timer init
		self.timerTimer = self.create_timer(0.5, self.timer_servos_callback)

		# Timer value init
		self.timerValue = 0

	def callback_update_team(self, msg): self.team = str(msg.data)

	def callback_update_solar(self, msg): 
		if self.team in ['bleu', 'Bleu', 'BLEU', 'Blue', 'BLUE', 'blue']:
			self.servoCmd[9] = 0
			if msg.data:
				self.servoCmd[9] = 180
		elif self.team in ['jaune', 'Jaune', 'JAUNE', 'Yellow', 'yellow', 'YELLOW']:
			self.servoCmd[9] = 180
			if msg.data:
				self.servoCmd[9] = 0
		else:
			self.servoCmd[9] = 90


	def callback_update_servos(self,msg):
		servoCmdMsg = msg.data
		servoCmdMsg1 = servoCmdMsg[0]
		match int(servoCmdMsg1):
			# Ouvre les pinces avec les plantes avec la pince en pos basse
			case 0:
				self.servoCmd[0] = int(self.posElev[1])
				self.servoCmd[1] = int(self.posPick[2][0])
				self.servoCmd[2] = int(self.posPick[2][1])
			# Ouvre les pinces avec les pot avec la pince en pos basse
			case 1:
				self.servoCmd[0] = int(self.posElev[0])
				self.servoCmd[1] = int(self.posPick[2][0])
				self.servoCmd[2] = int(self.posPick[2][1])
			# Ferme les pinces avec les plantes avec la pince en pos basse
			case 2:
				self.servoCmd[0] = int(self.posElev[0])
				self.servoCmd[1] = int(self.posPick[0][0])
				self.servoCmd[2] = int(self.posPick[0][1])
			# Ferme les pinces avec les pot avec la pince en pos basse
			case 3:
				self.servoCmd[0] = int(self.posElev[0])
				self.servoCmd[1] = int(self.posPick[1][0])
				self.servoCmd[2] = int(self.posPick[1][1])
			# Ferme la pince & transporte les pots en position haute
			case 4:
				self.servoCmd[0] = int(self.posElev[3])
				self.servoCmd[1] = int(self.posPick[0][0])
				self.servoCmd[2] = int(self.posPick[0][1])
			# Laisser tomber les pots dans la jardiniere
			case 5:
				self.servoCmd[0] = int(self.posElev[2])
				self.servoCmd[1] = int(self.posPick[2][0])
				self.servoCmd[2] = int(self.posPick[2][1])
			#Position de repos bas
			case 6:
				self.servoCmd[0] = int(self.posElev[0])
				self.servoCmd[1] = int(self.posPick[3][0])
				self.servoCmd[2] = int(self.posPick[3][1])
			#Position de repos haut
			case _:
				self.servoCmd[0] = int(self.posElev[3])
				self.servoCmd[1] = int(self.posPick[3][0])
				self.servoCmd[2] = int(self.posPick[3][1])

		servoCmdMsg2 = servoCmdMsg[1]
		match int(servoCmdMsg2):
			# Ouvert bas plante
			case 0:
				self.servoCmd[4] = int(self.posElev[1])
				self.servoCmd[5] = int(self.posPick[2][0])
				self.servoCmd[6] = int(self.posPick[2][1])
			# Ouvert bas pot
			case 1:
				self.servoCmd[4] = int(self.posElev[0])
				self.servoCmd[5] = int(self.posPick[2][0])
				self.servoCmd[6] = int(self.posPick[2][1])
			# Ferme bas plante 
			case 2:
				self.servoCmd[4] = int(self.posElev[0])
				self.servoCmd[5] = int(self.posPick[0][0])
				self.servoCmd[6] = int(self.posPick[0][1])
			# Ferme bas pot
			case 3:
				self.servoCmd[4] = int(self.posElev[0])
				self.servoCmd[5] = int(self.posPick[1][0])
				self.servoCmd[6] = int(self.posPick[1][1])
			# Ferme transport
			case 4:
				self.servoCmd[4] = int(self.posElev[3])
				self.servoCmd[5] = int(self.posPick[0][0])
				self.servoCmd[6] = int(self.posPick[0][1])
			# Laisse tomber
			case 5:
				self.servoCmd[4] = int(self.posElev[2])
				self.servoCmd[5] = int(self.posPick[2][0])
				self.servoCmd[6] = int(self.posPick[2][1])
			#Position de repos bas
			case 6:
				self.servoCmd[4] = int(self.posElev[0])
				self.servoCmd[5] = int(self.posPick[3][0])
				self.servoCmd[6] = int(self.posPick[3][1])
			#Position de repos haut
			case _:
				self.servoCmd[4] = int(self.posElev[3])
				self.servoCmd[5] = int(self.posPick[3][0])
				self.servoCmd[6] = int(self.posPick[3][1])

		servoCmdMsg3 = servoCmdMsg[2] #pos bras panneaux solaire
		match int(servoCmdMsg3):
			case 0:
				self.servoCmd[8] = 0
			case 1:
				self.servoCmd[8] = 75
			case 2:
				self.servoCmd[8] = 60
			case _:
				self.servoCmd[8] = 0




	def timer_servos_callback(self):
		msgToSend = Int16MultiArray()
		msgToSend.data = self.servoCmd
		self.publisherServos.publish(msgToSend)

def main(args=None):
	rclpy.init(args=args)
	node = servoPrepareNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
