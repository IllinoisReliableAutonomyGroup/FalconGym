import cv2
import os
import numpy as np
import torch
from ns_renderer_4_gates import SplatRenderer
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R 

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# model_path = "./outputs/0413_SP_analog_2/splatfacto/2025-04-19_123912/"
model_path = "./outputs/0413_SP_iPhone/splatfacto/2025-04-23_225512/"
out_img = os.path.join("images/","combo.png")

scale_factor = 1
width, height = 1280, 720

    # "w": 1280,
    # "h": 720,
    # "fl_x": 1008.57564,
    # "fl_y": 1014.23455,
    # "cx": 662.151392,
    # "cy": 245.460335,

renderer = SplatRenderer(
    model_path+"config.yml",
    model_path+"dataparser_transforms.json",
    # width = width//scale_factor,
    # height = height//scale_factor,
    # fx = 1008.57564/scale_factor, 
    # fy = 1014.23455/scale_factor,
    # cx = 662.151392/scale_factor,
    # cy = 245.460335/scale_factor

    width = 640,
    height = 480,
    fx = 459.3865864461926,
    fy = 457.4258819092027,
    cx = 314.2217566212822,
    cy = 235.34699554234373,
)


with open('fixed_wing_traj.txt', 'r') as f:
    data = f.readlines()
    

for idx, log in enumerate(data):
    px, py, pz, yaw, pitch, roll = [float(i) for i in log.split(" ")][:6]
    pitch *= -1 # dynamics specific



    # pose = np.array([1.85, -20, 4, 1.57])
    # px, py, pz, yaw = pose
    # Step 1: Create the rotation matrix from pitch, yaw, roll
    rot = R.from_euler("ZYX", (yaw, pitch, roll))

    # Get the 3x3 rotation matrix
    R_matrix = rot.as_matrix()

    # Step 2: Create the 4x4 transformation matrix
    camera_pose = np.eye(4)  # Start with an identity matrix
    camera_pose[:3, :3] = R_matrix  # Set the top-left 3x3 to the rotation matrix

    camera_pose[:3, 3] = [px,py,pz]  # Set the translation components

    img = renderer.render(camera_pose) # RGB
    cv_img = img[:, :, ::-1]
    cv2.imwrite(f'visualize_trajectory_testing/{idx:04}.png', cv_img)

    # plt.imshow(img)
    # plt.show()
    # cv2.imshow("demo", cv_img)
    # cv2.waitKey(0)

    # # closing all open windows
    # cv2.destroyAllWindows()