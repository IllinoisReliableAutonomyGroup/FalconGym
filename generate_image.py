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
#     k1 = 0.14467698542743748,
#     k2 = -0.0451994205785643,
#     p1 = 0.037891706600157816,
#     p2 = 0.05022987302713324,


    # width = 1080,
    # height = 1920,
    # fx = 1623.5967830411732,
    # fy = 1623.1742756141505,
    # cx = 541.0356265414346,
    # cy = 961.9680819558206,
)


# Digital 
pose = np.array([2, -15, 2, 1.57, 0, 0])
pose = np.array([2.5, 0, -0.5, 1.57, 0, 0])

# # Analog Pose
# pose = np.array([-2, -26, -0.8, 1.75, 0, 0])
# pose = np.array([-2.89, -24.38, -0.3889, 1.69, -0.1579, 0.2188 -0.1])
# pose = np.array([-2.34, -12.654, -1.69, 1.64, -0.1698, -0.0113 - 0.1 ])
px, py, pz, yaw, pitch, roll = pose
# pz = max(pz, -1.25)

# Digital
rot = R.from_euler("ZYX", (yaw, pitch, roll))

# Analog
# rot = R.from_euler("ZYX", (yaw, 0.1, 0.15))

# Get the 3x3 rotation matrix
R_matrix = rot.as_matrix()

# Step 2: Create the 4x4 transformation matrix
camera_pose = np.eye(4)  # Start with an identity matrix
camera_pose[:3, :3] = R_matrix  # Set the top-left 3x3 to the rotation matrix

camera_pose[:3, 3] = [px,py,pz]  # Set the translation components

# camera_pose = np.array(
#     [
#                 [
#                     -0.8455516393906483,
#                     0.008954817206546445,
#                     0.5338185425522284,
#                     15.369956340527926
#                 ],
#                 [
#                     -0.5279079188331369,
#                     -0.13527839111752063,
#                     -0.8384586967346229,
#                     -4.655013011850592
#                 ],
#                 [
#                     -0.07972235794966309,
#                     -0.9907671614185912,
#                     -0.1096575464685097,
#                     -2.662637496056275
#                 ],
#                 [
#                     0.0,
#                     0.0,
#                     0.0,
#                     1.0
#                 ]
#             ],
# )

img = renderer.render(camera_pose) # RGB
cv_img = img[:, :, ::-1]
cv2.imwrite(out_img, cv_img)

plt.imshow(img)
plt.show()
# cv2.imshow("demo", cv_img)
# cv2.waitKey(0)

# # closing all open windows
# cv2.destroyAllWindows()