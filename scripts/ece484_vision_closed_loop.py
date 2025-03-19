"""
File: ece484_vision_closed_loop.py

Description: This script simulates a drone's trajectory on a specified track using closed-loop control through vision.

Tracks supported: Circle_Track, Uturn_Track, Lemniscate_Track

The simulation involves rendering images from NerfRenderer class[ns-renderer.py], 
saves the trajectory to a text file, and creating a video from the rendered images.
"""
import numpy as np
from drone_dynamics import drone_dynamics
from ece484_vision_controller import vision_controller
import torch 
from nerfstudio.models.splatfacto import SplatfactoModel
from scipy.spatial.transform import Rotation as R 
import cv2 
from nerfstudio.cameras.cameras import Cameras, CameraType
from nerfstudio.utils.eval_utils import eval_setup
from nerfstudio.utils import colormaps
import numpy as np 
import os 
from pathlib import Path
import matplotlib.pyplot as plt 
import json
from scipy.spatial.transform import Rotation
import time
import pickle
import random
from ns-renderer import NerfRenderer


def save_trajectory_to_txt(trajectory, filename="trajectory.txt"):
    """Save trajectory data to a TXT file"""
    with open(filename, mode='w') as file: 
        for row in trajectory:
            file.write(" ".join(map(str, row)) + "\n")

def parse_args():
    parser = argparse.ArgumentParser(description="Closed loop Vision controller ")
    parser.add_argument("--track_name", type=str, default="circle",
                        help="give track name")

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    track_path = args.track_path
    scale_factor = 1
    width, height = 640, 480
    fov = 80
    fx = (width/2)/(np.tan(np.deg2rad(fov)/2))
    fy = (height/2)/(np.tan(np.deg2rad(fov)/2))

    renderer = NerfRenderer(

        '../outputs/'+track_path+'/nerfacto/'+track_path+'/config.yml',
        width = width//scale_factor,
        height = height//scale_factor,
        fx = 546.84164912/scale_factor, 
        fy = 547.57957461/scale_factor,
        cx = 349.18316327/scale_factor,
        cy = 215.54486004/scale_factor,
        track = track_path
    )

    def find_angle_difference(angle1, angle2):
        return (angle1 - angle2 + np.pi) % (2 * np.pi) - np.pi

    import numpy as np
    from scipy.spatial.transform import Rotation as R
    import pickle


    test_steps = 1000
    cur_pos = [1.4, -0.9, 0.42, 0, 0, -2] # x, y, z, roll, pitch, yaw, NOTE: this yaw is set to the GT x-y cat coord
    vx, vy, vz = -0.2, -0.2, 0
    x, y, z, roll, pitch, yaw = cur_pos
    control_val = [0, 0, 0, 0] # ax, ay, az, yaw_rate

    trajectory = []
    imagelist = []

    dynamics_state = [x, y, z, vx, vy, vz, yaw] # x, y, z, vx, vy, vz, cur_yaw

    for idx in range(test_steps):

        # Step 1: Create the rotation matrix from pitch, yaw, roll
        rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)  # 'xyz' assumes roll, pitch, yaw order
        rotation_matrix = rotation.as_matrix()  # Converts to a 3x3 rotation matrix

        # Step 2: Create the 4x4 transformation matrix
        camera_pose = np.eye(4)  # Start with an identity matrix
        camera_pose[:3, :3] = rotation_matrix  # Set the top-left 3x3 to the rotation matrix
        camera_pose[:3, 3] = [x, y, z]  # Set the translation components

        s = time.time()
        img = renderer.render(camera_pose) # RGB
        cv_img = img[:, :, ::-1]
        print(f"Total Inference taken: {round(time.time() - s, 4)}s\n")
        
        control = vision_controller(img)
        next_state = drone_dynamics(dynamics_state, control) 
        x, y, z, vx, vy, vz, yaw = next_state

        trajectory.append(x,y,z,yaw)
        imagelist.append(cv_img)
        
    
    for idx in range(test_steps):#saving images
        cv2.imwrite(f"./closed_loop/images/{idx:04d}.png", cv_img)

    save_trajectory_to_txt(trajectory, filename=track + "_vision_trajectory.txt")
    print(f"Trajectory saved to {track}_trajectory.txt")


