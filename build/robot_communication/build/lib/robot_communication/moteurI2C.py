import time
import random
import busio
import board
import sys
import RPi.GPIO as GPIO
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int16
from std_msgs.msg import String

class moteurI2C(Node):
    def __init__(self):
        super().__init__('moteurI2C')

        self.reset_pic = 4
        self.pic1_add = 0x51
        self.i2c = busio.I2C(board.SCL, board.SDA, 100000)
        self.result = bytearray(1)
        self.motorCmdSpeed = 0

        # ROS topics publisher declare
        self.publisher_encoder = self.create_publisher(Int16, '/encoder', 10)
        self.subscriber_motor = self.create_subscription(Int16, '/motor', self.callback_update_motor, 10)
        
        # Timer and callback declaration
        self.timer_encoder = self.create_timer(0.1, self.timer_encoder_callback)

        # GPIO init and reset PIC procedure
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.reset_pic, GPIO.OUT)
        GPIO.output(self.reset_pic, GPIO.LOW)
        time.sleep(.5)
        GPIO.output(self.reset_pic, GPIO.HIGH)

        if not self.pic1_add in self.i2c.scan():
            #self.get_logger().info("PIC not found at address " + self.pic1_add)
            sys.exit()
        
    def callback_update_motor(self, msg):
        self.motorCmdSpeed = msg.data
        self.get_logger().info(f"received : {self.motorCmdSpeed}")
        self.motorCmdSpeed = self.motorCmdSpeed*(-1)**(self.motorCmdSpeed<0)+128*(self.motorCmdSpeed<0)
        self.get_logger().info(f"cmd : {self.motorCmdSpeed}")
        self.i2c.writeto(self.pic1_add, bytes([self.motorCmdSpeed]))

    def timer_encoder_callback(self):
        msg = Int16()
        self.i2c.readfrom_into(self.pic1_add, self.result)
        self.get_logger().info(f"reading encoder : {self.result}")
        #msg.data = self.result


def signalHandler(sig, frame):
	# Function called when typing ctrl-c -> send 0 cmd to the motor
	i2c.writeto(0x53, bytes([0]))
	sys.exit(0)

def main(args=None):
    signal.signal(signal.SIGINT,signalHandler)
    rclpy.init(args=args)
    node = testI2C()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
