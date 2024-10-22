import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import time
from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from luma.led_matrix.device import max7219

class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller')
        self.serial = spi(port=0, device=1, gpio=noop())
        self.device = max7219(self.serial, cascaded=1)
        self.subscription = None
        self.led_states = {}
        self.load_configuration()

    def load_configuration(self):
        with open('led_config.yaml', 'r') as config_file:
            config = yaml.safe_load(config_file)
            for topic, params in config.items():
                self.led_states[topic] = {'led_id': params['led_id'], 'on_message': params['on_message'],
                                          'off_message': params['off_message'], 'delay': params['delay']}

    def subscribe_to_topics(self):
        for topic in self.led_states.keys():
            self.subscription = self.create_subscription(String, topic, self.message_callback, 10)

    def message_callback(self, msg):
        topic = self.get_topic_name()
        if topic in self.led_states:
            led_id = self.led_states[topic]['led_id']
            on_message = self.led_states[topic]['on_message']
            off_message = self.led_states[topic]['off_message']
            delay = self.led_states[topic]['delay']
            if msg.data == on_message:
                self.turn_on_led(led_id)
            elif msg.data == off_message:
                self.turn_off_led(led_id, delay)

    def turn_on_led(self, led_id):
        with canvas(self.device) as draw:
            draw.point((led_id, 0), fill="white")

    def turn_off_led(self, led_id, delay):
        time.sleep(delay)
        with canvas(self.device) as draw:
            draw.point((led_id, 0), fill="black")

def main():
    rclpy.init()
    controller = LEDController()
    controller.subscribe_to_topics()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
