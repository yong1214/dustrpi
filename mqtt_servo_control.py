import paho.mqtt.client as mqtt
from gpiozero import Servo
from time import sleep

# MQTT settings
BROKER = "localhost"  # Use "broker.hivemq.com" for public broker or your public IP
PORT = 1883
TOPIC = "pi/servo1/angle"
USERNAME = "madmax"  # Set if using authentication
PASSWORD = "HomeGuard@123"  # Set if using authentication

# Initialize servo
servo = Servo(18)

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
        client.subscribe(TOPIC)
    else:
        print(f"Connection failed with code {rc}")

def on_message(client, userdata, msg):
    try:
        angle = float(msg.payload.decode())
        if 0 <= angle <= 180:
            servo.value = (angle - 90) / 90  # Map 0-180° to -1 to 1
            print(f"Set servo to {angle} degrees")
        else:
            print("Angle out of range (0-180)")
    except ValueError:
        print("Invalid angle received")

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
    client.disconnect()
    print("Program stopped")
