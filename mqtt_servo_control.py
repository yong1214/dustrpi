import paho.mqtt.client as mqtt
import lgpio
from gpiozero import Servo
from time import sleep
import sys

if len(sys.argv) != 3:
    print("Usage: python3 mqtt_servo_control.py <PORT> <GPIO>")
    sys.exit(1)

MQTT_PORT = sys.argv[1]
SERVO_PIN = sys.argv[2]

# PWM output settings
SERVO_LOW = 500 # 0 degree
SERVO_MID = 1500 # 135 degree
SERVO_HIGH = 2500 # 270 degree
SERVO_PWM_FREQ = 100
SERVO_PWM_OFFSET = 0
SERVO_PWM_CYCLE = 1
 
# MQTT settings
BROKER = "localhost"  # Use "broker.hivemq.com" for public broker or your public IP
TOPIC = "pi/servo1/angle"
USERNAME = "madmax"  # Set if using authentication
PASSWORD = "HomeGuard@123"  # Set if using authentication

chip_handle = -1

def init_servo_gpio():
    global chip_handle

    # Open the GPIO chip (e.g., /dev/gpiochip4 on Raspberry Pi 5)
    chip_handle = lgpio.gpiochip_open(4)  # Get the gpiochip
    if chip_handle >= 0:
        gpio_pin = int(SERVO_PIN)
        result = lgpio.gpio_claim_output(chip_handle, gpio_pin, lFlags=lgpio.SET_PULL_DOWN)
        if result < 0:
            print(f"Error claiming output pin with pull-down: {lgpio.error_text(result)}")
        else:
            print(f"Successfully claimed GPIO {gpio_pin} as output with pull-down")
        # Add a delay to allow lgpio configuration to take effect
        sleep(1)
    else:
        print(f"Error opening GPIO chip: {lgpio.error_text(chip_handle)}")

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
        client.subscribe(TOPIC)
    else:
        print(f"Connection failed with code {rc}")

def on_message(client, userdata, msg):
    global chip_handle
    global SERVO_PIN
    global SERVO_PWM_FREQ
    global SERVO_PWM_OFFSET
    global SERVO_PWM_CYCLE
    try:
        angle = float(msg.payload.decode())
        if 0 <= angle <= 270:
            pwm_wd = int((angle / 270) * 2000 + 500)  # Convert the degree rotation to PWM pulse
            lgpio.tx_servo(chip_handle, int(SERVO_PIN), pwm_wd, SERVO_PWM_FREQ, SERVO_PWM_OFFSET, SERVO_PWM_CYCLE)
            print(f"Set servo to {angle} degrees")
        else:
            print("Angle out of range (0-270)")
    except ValueError:
        print("Invalid angle received")

# Set up MQTT client
client = mqtt.Client()
if USERNAME and PASSWORD:
    client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect
client.on_message = on_message

# Connect to broker
client.connect(BROKER, int(MQTT_PORT), 60)

# Start main process
try:
    # Open gpio chip and set gpio PULL DOWN
    init_servo_gpio()
    # Reset servo gear position to LOW
    lgpio.tx_servo(chip_handle, int(SERVO_PIN), SERVO_LOW, SERVO_PWM_FREQ, SERVO_PWM_OFFSET, SERVO_PWM_CYCLE)
    sleep(1)
    # Start loop
    client.loop_forever()
except KeyboardInterrupt:
    print("Program stopped")
except Exception as e:
    print(f"Error during lgpio config: {e}")
finally:
    if chip_handle >= 0:
        result = lgpio.gpiochip_close(chip_handle)
        if result == 0:
            print("GPIO chip closed successfully")
        else:
            print(f"Error closing GPIO chip: {lgpio.error_text(result)}")
    client.disconnect()
    print("Finalized:>")
