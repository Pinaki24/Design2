import cv2
import numpy as np
import math
import time
from dronekit import connect, VehicleMode

class PIController:
    def __init__(self, kp, ki, setpoint=0, output_limits=(None, None)):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.setpoint = setpoint  # Desired setpoint
        self._integral = 0
        self._last_error = None
        self._last_time = None
        self.output_limits = output_limits  # Tuple (min, max)

    def reset(self):
        self._integral = 0
        self._last_error = None
        self._last_time = None

    def compute(self, measurement, current_time):
        error = self.setpoint - measurement
        delta_time = current_time - self._last_time if self._last_time else 0
        self._last_time = current_time

        # Proportional term
        p_term = self.kp * error

        # Integral term
        if delta_time > 0:
            self._integral += error * delta_time
            i_term = self.ki * self._integral
        else:
            i_term = 0

        # Compute total output
        output = p_term + i_term

        # Apply output limits
        min_output, max_output = self.output_limits
        if min_output is not None:
            output = max(min_output, output)
        if max_output is not None:
            output = min(max_output, output)

        return output

from pymavlink import mavutil
