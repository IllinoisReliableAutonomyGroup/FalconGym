"""
File: ece484_state_controller.py
Description: A Python module implementing a state controller for a drone. This script manages drone movements by controlling acceleration and yaw rate based on the current state of the drone.

This module provides functions to calculate control inputs (acceleration in x, y, z axes and yaw rate) based on the drone's current position, velocity, and orientation.
"""
import numpy as np
from ece484_state_controller import state_controller

def state_controller(state):
    """
    Calculates control inputs for a drone based on its current state.

    Parameters:
    - state (list): A list containing the drone's current state. It should include:
        - x (float): The x-coordinate of the drone's position.
        - y (float): The y-coordinate of the drone's position.
        - z (float): The z-coordinate of the drone's position.
        - vx (float): The x-component of the drone's velocity.
        - vy (float): The y-component of the drone's velocity.
        - vz (float): The z-component of the drone's velocity.
        - yaw (float): The yaw angle of the drone.

    Returns:
    - control (list): A list containing the control inputs for the drone. It includes:
        - ax (float): The desired acceleration in the x-axis.
        - ay (float): The desired acceleration in the y-axis.
        - az (float): The desired acceleration in the z-axis.
        - yaw_rate (float): The desired yaw rate.

    TODO TASK 1
    Implementation Notes:
    - This function should implement a control logic that calculates the desired accelerations and yaw rate based on the drone's current state.
    - The control logic might involve PID controllers, model predictive control, or other control strategies.
    """
    control = [0, 0, 0 , 0] # ax, ay, az, yaw_rate
    return control

if __name__ == "__main__":
    state = [0, 0, 1, 0, 0, 0, 0] # x, y, z, vx, vy, vz, yaw
    control = state_controller(state)
    print(control)