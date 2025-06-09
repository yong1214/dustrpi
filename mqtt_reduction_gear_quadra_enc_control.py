import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from time import sleep

# MQTT settings
BROKER = "localhost"  # Use "broker.hivemq.com" for public broker or your public IP
PORT = 1883
TOPIC = "pi/dcmotor/control"
USERNAME = "madmax"  # Set if using authentication
PASSWORD = "HomeGuard@123"  # Set if using authentication

# GPIO setup
GPIO.setmode(GPIO.BCM)
ENABLE_PIN = 25  # PWM for speed
INPUT1_PIN = 24  # Direction
INPUT2_PIN = 23  # Direction
GPIO.setup(ENABLE_PIN, GPIO.OUT)
GPIO.setup(INPUT1_PIN, GPIO.OUT)
GPIO.setup(INPUT2_PIN, GPIO.OUT)
pwm = GPIO.PWM(ENABLE_PIN, 100)  # 100 Hz
pwm.start(0)

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
        client.subscribe(TOPIC)
    else:
        print(f"Connection failed with code {rc}")

def on_message(client, userdata, msg):
    try:
        command = msg.payload.decode().split(",")
        direction = command[0].lower()
        speed = float(command[1])
        
        if speed < 0 or speed > 100:
            print("Speed out of range (0-100)")
            return
        
        if direction == "forward":
            GPIO.output(INPUT1_PIN, GPIO.HIGH)
            GPIO.output(INPUT2_PIN, GPIO.LOW)
            pwm.ChangeDutyCycle(speed)
            print(f"Motor forward at {speed}%")
        elif direction == "reverse":
            GPIO.output(INPUT1_PIN, GPIO.LOW)
            GPIO.output(INPUT2_PIN, GPIO.HIGH)
            pwm.ChangeDutyCycle(speed)
            print(f"Motor reverse at {speed}%")
        elif direction == "stop":
            pwm.ChangeDutyCycle(0)
            print("Motor stopped")
        else:
            print("Invalid direction")
    except (ValueError, IndexError):
        print("Invalid command format. Use: direction,speed (e.g., forward,50)")

# Set up MQTT client
client = mqtt.Client()
if USERNAME and PASSWORD:
    client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect
client.on_message = on_message

# Connect to broker
client.connect(BROKER, PORT, 60)

# Start loop
try:
    client.loop_forever()
except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
    client.disconnect()
    print("Program stopped")

