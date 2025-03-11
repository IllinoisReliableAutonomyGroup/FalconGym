import numpy as np
from drone_dynamics import drone_dynamics
from ece484_state_controller import state_controller
import argparse
import json
import numpy as np


# read gate position input track name : Circle_Track, Uturn_Track, Lemniscate_Track  output : dictinary gate coordinates
def get_gate_coordinates(track_name):
    filename = "./scripts/gates_poses.json"

    with open(filename, 'r') as file:
        gate_data = json.load(file)
    
    if track_name in gate_data:
        return gate_data[track_name]
    else:
        raise ValueError(f"Track name '{track_name}' not found in JSON file.")

    
        

def parse_args():
    parser = argparse.ArgumentParser(description="NeRF Renderer for Camera Poses")
    parser.add_argument("--track-name", type=str, required=True,
                        help="Specify track name [Circle_Track, Uturn_Track, Lemniscate_Track ] ")

    return parser.parse_args()



if __name__ == "__main__":

    args = parse_args()
    track = args.track_name

    print(get_gate_coordinates(track))
   
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
    