# This Raspberry Pi code was developed by newbiely.com
# This Raspberry Pi code is made available for public use without any restriction
# For comprehensive instructions and wiring diagrams, please visit:
# https://newbiely.com/tutorials/raspberry-pi/raspberry-pi-led-matrix


from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.virtual import viewport, sevensegment
from time import sleep

CS_PIN = 25  # Replace with your actual CS pin
BLOCK_NUM = 4  # Replace with your block number

HEIGHT = 8
WIDTH = 8 * BLOCK_NUM

# Define SPI interface
serial = spi(port=0, device=1, gpio=noop(), cs=CS_PIN)

# Define LED matrix device
device = max7219(serial, cascaded=BLOCK_NUM, block_orientation=-90)

# Define virtual device
virtual = viewport(device, width=WIDTH, height=HEIGHT)

# Create instance of sevensegment for text display
ledMatrix = sevensegment(virtual)

def clear_display():
    ledMatrix.text = "        "
    sleep(1)

def display_text(text, alignment):
    ledMatrix.text = text
    ledMatrix.text_align = alignment
    sleep(2)


try:
    while True:
        clear_display()

        display_text("Left", "left")
        display_text("Center", "center")
        display_text("Right", "right")

        clear_display()

        display_text("Invert", "center")
        sleep(2)

        ledMatrix.text = "1234"
        sleep(2)

except KeyboardInterrupt:
    pass
finally:
    device.cleanup()

