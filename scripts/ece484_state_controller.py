import numpy as np

def state_controller(state):
    control = [0, 0, 0 , 0] # ax, ay, az, yaw_rate
    return control

if __name__ == "__main__":
    state = [0, 0, 1, 0, 0, 0, 0] # x, y, z, vx, vy, vz, yaw
    control = state_controller(state)
    print(control)