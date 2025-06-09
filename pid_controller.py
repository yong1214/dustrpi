import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update(self, setpoint, current_position):
        # Calculate error
        error = setpoint - current_position

        # Calculate proportional term
        proportional_output = self.kp * error

        # Calculate integral term
        self.integral += error * (time.time() - self.last_time)
        integral_output = self.ki * self.integral

        # Calculate derivative term
        derivative = (error - self.last_error) / (time.time() - self.last_time)
        derivative_output = self.kd * derivative

        # Update last error and time
        self.last_error = error
        self.last_time = time.time()

        # Calculate total PID output
        pid_output = proportional_output + integral_output + derivative_output

        return pid_output
