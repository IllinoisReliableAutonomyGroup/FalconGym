import numpy as np
from drone_dynamics import drone_dynamics
from ece484_state_controller import state_controller

# TODO: read gate position

if __name__ == "__main__":
    state = [0, 0, 1, 0, 0, 0, 0] # # x, y, z, vx, vy, vz, yaw
    
    # max step is 1000 (i.e. 1000*0.05 = 50s), you should be able to travel 2 laps within 50s
    cur_step = 0
    MAX_STEP = 1000
    trajectory = [state]
    if cur_step <= MAX_STEP:
        control = state_controller(state)
        state = drone_dynamics(state, control)
        trajectory.append(state) # N * 7 
    
    # TODO: log trajectory into a txt
