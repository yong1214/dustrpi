import paho.mqtt.client as mqtt
import lgpio
from pid_controller import PIDController
from time import sleep

# MQTT settings
BROKER = "localhost"  # Use "broker.hivemq.com" for public broker or your public IP
PORT = 1886
TOPIC = "pi/dcmotor/control"
USERNAME = "madmax"  # Set if using authentication
PASSWORD = "HomeGuard@123"  # Set if using authentication

# --- Configuration (Adjust to your setup) ---
MOTOR_ENA = 18    # Enable pin for motor driver
MOTOR_IN1 = 23    # Input 1 for motor driver
MOTOR_IN2 = 24    # Input 2 for motor driver
ENCODER_A = 17    # Encoder A phase pin
ENCODER_B = 27    # Encoder B phase pin

PWM_MIN = 45
PWM_MAX = 65
PWM_FREQUENCY = 1000 # PWM frequency in Hz (Adjust as needed)
# Counts per revolution after scaling >> PPR * 4 * Gear Ratio << for example, PPR = 4, gear ratio = 40:1, then CPR = 4 * 4 * 40 = 640
# For 60 RPM HD Premium Planetary Gear Motor w Encoder, PPR = 12  and Gear Ratio = 139:1, so CPR = 12 * 4 * 139 = 6672 
CNT_PER_REV = 6672
CNT_PER_REV_OFFSET = 0
REV_NUM = 2 # Number of rotations in a running cycle

# --- Initialize PID controller
Kp = 1  # Proportional gain
Ki = 1.5  # Integral gain
Kd = 0.02 # Derivative gain
pid = PIDController(Kp, Ki, Kd)

"""
Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
"""

# --- Motor Control Functions ---

def motor_forward(h):
    lgpio.gpio_write(h, MOTOR_IN1, 1)
    lgpio.gpio_write(h, MOTOR_IN2, 0)

def motor_backward(h):
    lgpio.gpio_write(h, MOTOR_IN1, 0)
    lgpio.gpio_write(h, MOTOR_IN2, 1)

def motor_stop(h):
    lgpio.gpio_write(h, MOTOR_IN1, 0)
    lgpio.gpio_write(h, MOTOR_IN2, 0)
    lgpio.tx_pwm(h, MOTOR_ENA, PWM_FREQUENCY, 0) # Turn off PWM

def set_speed(h, speed):
    """Sets the motor speed using PWM.
    Speed is a float between 0.0 and 1.0 (corresponds to duty cycle in %)
    """
    if speed < 0.0:
        speed = 0.0
    elif speed > 1.0:
        speed = 1.0
    # Scale speed to a 0-100 range for PWM duty cycle
    duty_cycle = int(speed * 100)
    lgpio.tx_pwm(h, MOTOR_ENA, PWM_FREQUENCY, duty_cycle)

# --- Encoder Handling ---

encoder_count = 0 # Global to hold encoder count
last_enc_a_level = 0
last_enc_b_level = 0
last_gpio = None

# Set desired position
target_count = 0 # Target position in encoder counts

last_switch = 2 # Store the last switch ON/OFF

def encoder_callback(chip, gpio, level, tick):
    global encoder_count
    global last_enc_a_level
    global last_enc_b_level
    global last_gpio

    if gpio == ENCODER_A:
        last_enc_a_level = level
    else:
        last_enc_b_level = level

    if gpio != last_gpio: # debounce
        last_gpio = gpio

        if gpio == ENCODER_A and level == 1:
            if last_enc_b_level == 1:
                encoder_count += 1
        elif gpio == ENCODER_B and level == 1:
            if last_enc_a_level == 1:
                encoder_count -= 1

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
        client.subscribe(TOPIC)
    else:
        print(f"Connection failed with code {rc}")

first_msg = True
def on_message(client, userdata, msg):
    global encoder_count
    global h
    global last_switch
    global first_msg
    
    if first_msg == True:
        first_msg = False
        return
    try:
        command = msg.payload.decode('utf-8')
        switch = int(command)

        if switch != 0 and switch != 1:
            print(f"The command is only valid with payload (0,1), received {command}")
            return
        #if switch == last_switch:
        #    printf("Repeated switch, ignored!")
        #    return
    except UnicodeDecodeError:
        print("Decode error")
        return

    target_count = CNT_PER_REV * REV_NUM
    encoder_count = 0
    last_encoder_count = 0
    pid_output = 0
    errors = 0
    MAX_ERR = 2
    if switch == 1:
        target_count = -target_count
    while abs(encoder_count) <=  abs(target_count):
        if encoder_count == 0 or last_encoder_count == encoder_count:
            errors += 1
            print(f"error: {errors}")
        if errors >= MAX_ERR:
            break
        pid_output = pid.update(abs(target_count), abs(encoder_count))
        print(f"PID output: {pid_output} Encoder: {encoder_count}")
        last_encoder_count = encoder_count
        if target_count < 0:
            motor_forward(h)
        else:
            motor_backward(h)
        # Limit output of PID Controller to valid PWM range
        pid_output_scaled = abs(pid_output) * round(100.0/abs(target_count), 2)
        pwm_output = max(PWM_MIN, min(PWM_MAX, pid_output_scaled))
        set_speed(h, pwm_output/100.0)
        print(f"PWM: {pwm_output:.2f}% Scaled PID output: {pid_output_scaled:.2f}")
        sleep(0.1)
    last_switch = switch
    print(f"Switched to {switch}")
    if h: 
        print("Stopping Motor...")
        motor_stop(h)
        set_speed(h, 0)
        sleep(0.1)

# Set up MQTT client
client = mqtt.Client()
if USERNAME and PASSWORD:
    client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect
client.on_message = on_message

# Connect to broker
client.connect(BROKER, PORT, 60)

# Main process
try:
    # Open GPIO Chip
    h = lgpio.gpiochip_open(4)

    # Set up Motor Pins
    lgpio.gpio_claim_output(h, MOTOR_ENA)
    lgpio.gpio_claim_output(h, MOTOR_IN1)
    lgpio.gpio_claim_output(h, MOTOR_IN2)

    # Set up Encoder Pins
    lgpio.gpio_claim_input(h, ENCODER_A)
    lgpio.gpio_claim_input(h, ENCODER_B)

    lgpio.gpio_claim_alert(h, ENCODER_A, lgpio.BOTH_EDGES)
    lgpio.gpio_claim_alert(h, ENCODER_B, lgpio.BOTH_EDGES)

    # Setup Encoder Interrupt
    cb_ena = lgpio.callback(h, ENCODER_A, lgpio.BOTH_EDGES, encoder_callback)
    cb_enb = lgpio.callback(h, ENCODER_B, lgpio.BOTH_EDGES, encoder_callback)
    sleep(10)

    # Start loop
    client.loop_forever()

except lgpio.error as e:
    print(f"Error: {e}")

except KeyboardInterrupt:
    client.disconnect()
    print("Program stopped")

finally:
    # Release resources
    if h:
      print("Stopping Motor...")
      motor_stop(h)
      set_speed(h, 0)
      sleep(0.1)
      cb_ena.cancel()
      cb_enb.cancel()
      lgpio.gpiochip_close(h)
    print("Finalized:>")
