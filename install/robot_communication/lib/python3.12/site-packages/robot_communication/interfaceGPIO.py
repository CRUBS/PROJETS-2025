###################################################
###						###
###	  Interface GPIO robot 2023-2024		###
###		CRUBS - Lorient			###
###	  club.robotique.ubs@gmail.com		###
###	   file creation 24/03/2024		###
###	last modification --/--/2024		###
###						###
###################################################

# Import libs
import time
from gpiozero import Button, OutputDevice

# Import libs ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# Configuration des GPIO
motorsResetPins = [17, 27, 22]
teamPin = 24
starterPin = 23

# GPIOzero setup
motors_reset_devices = [OutputDevice(pin, active_high=True, initial_value=True) for pin in motorsResetPins]
starter_button = Button(starterPin, bounce_time=0.3, pull_up=True)  # Utilisation de Button pour le starter
team_button = Button(teamPin, bounce_time=0.3, pull_up=True)  # Utilisation de Button pour le team pin

class gpioInterface(Node):
	def __init__(self):
		super().__init__('interfaceGPIO')

		# ROS2 timers
		self.timerTeam = self.create_timer(1, self.timer_team_callback)
		self.timerStarter = self.create_timer(0.25, self.timer_starter_callback)

		# ROS2 publisher
		self.publisherTeam = self.create_publisher(String, '/team', 10)
		self.publisherStarter = self.create_publisher(Bool, '/starterRemoved', 10)

		# ROS2 subscription
		self.subscriptionMotorState = self.create_subscription(String, '/timers_status/motors', self.callback_motors_state, 10)

	def timer_team_callback(self):
		msg = String()
		if not team_button.value == 1:
			msg.data = 'Jaune'
		else:
			msg.data = 'Bleu'
		self.publisherTeam.publish(msg)

	def timer_starter_callback(self):
		msg = Bool()
		if starter_button.value == 1:
			msg.data = True
		else:
			msg.data = False
			
		self.publisherStarter.publish(msg)

	def callback_motors_state(self, msg):
		if msg.data == 'error':
			self.get_logger().info("reset cartes moteurs")
			for motor in motors_reset_devices:
				motor.off()  # Reset: mettre à LOW
			time.sleep(0.05)
			for motor in motors_reset_devices:
				motor.on()  # Relever le reset: mettre à HIGH
			time.sleep(0.05)
		else:
			pass


def main(args=None):
	rclpy.init(args=args)
	node = gpioInterface()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
