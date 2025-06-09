import lgpio
import time
from pid_controller import PIDController
import sys

if len(sys.argv) != 2:
    print("Usage: python3 gear_quadra_enc__test.py <ON/OFF>")
    sys.exit(1)

target_position = sys.argv[1]
if target_position != 'ON' and target_position != 'OFF':
    print("Usage: python3 gear_quadra_enc_test.py <ON/OFF>")
    sys.exit(1)


# --- Configuration (Adjust to your setup) ---
MOTOR_ENA = 18    # Enable pin for motor driver
MOTOR_IN1 = 13    # Input 1 for motor driver
MOTOR_IN2 = 19    # Input 2 for motor driver
ENCODER_A = 17    # Encoder A phase pin
ENCODER_B = 27    # Encoder B phase pin

PWM_MIN = 45
PWM_MAX = 85
PWM_FREQUENCY = 1000 # PWM frequency in Hz (Adjust as needed)
CNT_PER_REV = 640 # Counts per revolution after scaling 4 CPR * 4 * 40:1 Gear Ratio
CNT_PER_REV_OFFSET = 0
REV_NUM = 6 # Number of rotations in a running cycle

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

# --- Main Process ---

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
    time.sleep(0.1)

    target_count = CNT_PER_REV * REV_NUM
    encoder_count = 0
    running = True
    if target_position == 'ON':
        target_count = -target_count
    while running == True:

        print(f"Gear Rotation {target_position} with PID Controller")
        running = False

        motor_stop(h)
        set_speed(h, 0)
        time.sleep(3)

        encoder_count = 0
        pid_output = 0
        while abs(encoder_count) <=  abs(target_count):
            pid_output = pid.update(abs(target_count), abs(encoder_count))
            print(f"PID output: {pid_output} Encoder: {encoder_count}")
            if target_count < 0:
                motor_forward(h)
            else:
                motor_backward(h)
            # Limit output of PID Controller to valid PWM range
            pid_output_scaled = abs(pid_output) * round(100.0/abs(target_count), 2)
            pwm_output = max(PWM_MIN, min(PWM_MAX, pid_output_scaled))
            set_speed(h, pwm_output/100.0)
            print(f"PWM: {pwm_output:.2f}% Scaled PID output: {pid_output_scaled:.2f}")
            time.sleep(0.1)

except KeyboardInterrupt:
    print("Program terminated:O")

except lgpio.error as e:
    print(f"Error: {e}")

finally:
    # Release resources
    if h:
      print("Stopping Motor...")
      motor_stop(h)
      set_speed(h, 0)
      time.sleep(0.1)
      cb_ena.cancel()
      cb_enb.cancel()
      lgpio.gpiochip_close(h)
    print("Finalized:>")

