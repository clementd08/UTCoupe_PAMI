# implement a simple PID controller

import matplotlib.pyplot as plt
import numpy as np
import time

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral = 0
        self.previous_error = 0

        self.error_curve = []

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        self.error_curve.append(error)

        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
    
    def reset(self):
        self.integral = 0
        self.previous_error = 0