
import numpy as np
from drone_dynamics import drone_dynamics
from ece484_vision_controller import vision_controller

import torch 
from nerfstudio.models.splatfacto import SplatfactoModel
# from nerfstudio.utils import load_config, load_model
# from nerfstudio
from scipy.spatial.transform import Rotation as R 
import cv2 
from nerfstudio.cameras.cameras import Cameras, CameraType
from nerfstudio.utils.eval_utils import eval_setup
from nerfstudio.utils import colormaps
import numpy as np 
import os 
from pathlib import Path
import matplotlib.pyplot as plt 
# from ns_dp_info import dpDict
import json
from scipy.spatial.transform import Rotation

import time
import pickle
import random

class NerfRenderer:
    def __init__(
        self,
        config_path: str, 
        width: int,
        height: int, 
        fx: float, 
        fy: float, 
        distortion_params: np.ndarray = None,
        camera_type = CameraType.PERSPECTIVE,
        metadata = None, 
        cx = None,
        cy = None
    ):
        self._script_dir = os.path.dirname(os.path.realpath(__file__))
        self.config_path = Path(os.path.join(self._script_dir, config_path))

        self.fx = fx
        self.fy = fy

        if cx:
            self.cx = cx
        else:
            self.cx = width/2
        if cy:
            self.cy = cy
        else:
            self.cy = height/2
        # self.cx, self.cy = width/2, height/2

        self.nerfW = width
        self.nerfH = height
        self.distortion_params = distortion_params
        self.camera_type  = camera_type

        self.focal = self.fx

        self.metadata = metadata

        _, pipeline, _, step = eval_setup(
            self.config_path,
            eval_num_rays_per_chunk=None,
            test_mode='inference'
        )
        self.model = pipeline.model 

        # 
        with open("./outputs/circle/nerf/circle/dataparser_transforms.json", 'r') as f:
            self.dp_trans_info = json.load(f)

    def render(self, cam_state:np.ndarray):
        # rpy = R.from_matrix(cam_state[0, :3,:3])
        
        # Convert camera pose to what's stated in transforms_orig.json
        tmp = Rotation.from_euler('zyx',[-np.pi/2,np.pi/2,0]).as_matrix()
        mat = cam_state[:3,:3]@tmp 
        cam_state[:3,:3] = mat 

        # # Convert camera pose to Colmap frame in transforms.json
        cam_state[0:3,1:3] *= -1
        cam_state = cam_state[np.array([0,2,1,3]),:]
        cam_state[2,:] *= -1 
        
        
        transform = np.array(self.dp_trans_info['transform'])
        scale_factor = self.dp_trans_info['scale']
        # print(transform.shape, cam_state.shape)
        cam_state = transform@cam_state
        cam_state[:3,3] *= scale_factor
        cam_state = cam_state[:3,:]

        if cam_state.ndim == 2:
            cam_state = np.expand_dims(cam_state, axis=0)
        camera_to_world = torch.FloatTensor( cam_state )

        camera = Cameras(camera_to_worlds = camera_to_world, fx = self.fx, fy = self.fy, cx = self.cx, cy = self.cy, width=self.nerfW, height=self.nerfH, distortion_params=self.distortion_params, camera_type=self.camera_type, metadata=self.metadata)
        camera = camera.to('cuda')
        
        ray_bundle = camera.generate_rays(camera_indices=0, aabb_box=None)
        
        with torch.no_grad():
            tmp = self.model.get_outputs_for_camera_ray_bundle(ray_bundle)
        s = time.time()
        

        img = tmp['rgb']
        # print(tmp)
        img =(colormaps.apply_colormap(image=img, colormap_options=colormaps.ColormapOptions(colormap='gray', normalize=True, colormap_min=0, colormap_max=255) )).cpu().numpy()
        # img = img.cpu().numpy()
        print(f"NN Inference setup time: {round(time.time() - s, 4)}s")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = (img * 255).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img

if __name__ == "__main__":
    scale_factor = 1
    width, height = 640, 480
    fov = 80
    fx = (width/2)/(np.tan(np.deg2rad(fov)/2))
    fy = (height/2)/(np.tan(np.deg2rad(fov)/2))

    renderer = NerfRenderer(

        '../outputs/circle/nerf/circle/config.yml',
        width = width//scale_factor,
        height = height//scale_factor,
        fx = 546.84164912/scale_factor, 
        fy = 547.57957461/scale_factor,
        # fx = fx,
        # fy = fy,
        cx = 349.18316327/scale_factor,
        cy = 215.54486004/scale_factor
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
    image_list = []

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

        with open('./closed_loop/circle_traj.txt', 'a') as f:
            f.write(f"{x} {y} {z} {yaw}\n")

        # plt.imshow(img)
        # plt.show()
        # break
        cv2.imwrite(f"./closed_loop/images/{idx:04d}.png", cv_img)
