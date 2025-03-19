"""
File: ece484_vision_controller.py
Description: A Python module implementing a vision controller for a drone. This script manages drone movements by controlling acceleration and yaw rate based on the fpv image of the drone.

This module provides functions to calculate control inputs (acceleration in x, y, z axes and yaw rate) based on the drone's current FPV.
"""

import numpy as np
from ece484_gate_detection import gate_detection

def vision_controller(image):
    """
    Calculates control inputs for a drone based on its current state.

    Parameters:
    - image (FPV drone view fron nerf)

    Returns:
    - control (list): A list containing the control inputs for the drone. It includes:
        - ax (float): The desired acceleration in the x-axis.
        - ay (float): The desired acceleration in the y-axis.
        - az (float): The desired acceleration in the z-axis.
        - yaw_rate (float): The desired yaw rate.

    TODO TASK 2
    Implementation Notes:
    - This function should implement a control logic that calculates the desired accelerations and yaw rate based on the drone's current state.
    - YOu can use state controller from ece484_state_controller.py along with gate detection.
    """
    control = [0, 0, 0 , 0] # ax, ay, az, yaw_rate
    return control

if __name__ == "__main__":
    image = np.zeros((480, 640, 3)) # PlaceHolder
    control = vision_controller(image)
    print(control)