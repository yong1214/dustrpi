#!/usr/bin/env python3
import lgpio
from gpiozero import DigitalOutputDevice, AngularServo
from gpiozero.pins.lgpio import LGPIOFactory
from time import sleep

# Specify GPIO pin and 
servo_pin = 12  # Replace with the actual GPIO pin connected to the servo
try:
    # Open the GPIO chip (e.g., /dev/gpiochip4 on Raspberry Pi 5)
    chip_handle = lgpio.gpiochip_open(4)  # Get the gpiochip
    if chip_handle >= 0:
        result = lgpio.gpio_claim_output(chip_handle, servo_pin, lFlags=lgpio.SET_PULL_DOWN)
        if result < 0:
            print(f"Error claiming output pin with pull-down: {lgpio.error_text(result)}")
        else:
            print(f"Successfully claimed GPIO {servo_pin} as output with pull-down")
        # Add a delay to allow lgpio configuration to take effect
        sleep(1)
    else:
        print(f"Error opening GPIO chip: {lgpio.error_text(chip_handle)}")

    low = 500 # 0 degree
    mid = 1500 # 135 degree for 20kg servo
    high = 2500 # 270 degree for 20kg servo
    freq = 100
    offset = 0
    cycle = 1
    step = 100
    pwm_wd = low
    lgpio.tx_servo(chip_handle, servo_pin, pwm_wd, freq, offset, cycle)
    sleep(2)
    while True:
        while pwm_wd < high:
            lgpio.tx_servo(chip_handle, servo_pin, pwm_wd, freq, offset, cycle)
            pwm_wd = pwm_wd + step
            sleep(2)
        pwm_wd = high
        while pwm_wd > low:
            lgpio.tx_servo(chip_handle, servo_pin, pwm_wd, freq, offset, cycle)
            pwm_wd = pwm_wd - step
            sleep(2)
        pwm_wd = low
        #lgpio.tx_servo(chip_handle, servo_pin, mid, freq, offset, cycle)
        #sleep(2)
        #lgpio.tx_servo(chip_handle, servo_pin, high, freq, offset, cycle)
        #sleep(2)
except KeyboardInterrupt:
    print("Program interrupted")
except Exception as e:
    print(f"Error during lgpio config: {e}")
finally:
    if chip_handle >= 0:
        result = lgpio.gpiochip_close(chip_handle)
        if result == 0:
            print("GPIO chip closed successfully")
        else:
            print(f"Error closing GPIO chip: {lgpio.error_text(result)}")
    print("Finalized:>")
