"""
File: ns-renderer.py

NeRF Renderer for Camera Poses

Description: This script provides a class-based implementation for rendering NeRF scenes based on given camera poses.
It utilizes the Nerfstudio framework to handle camera transformations and image rendering.

### Key Features:
- **Camera Pose Transformation**: Converts camera poses into the required format for rendering.
- **NeRF Rendering**: Uses a Splatfacto model to generate images from camera poses.

USE this script to generate training data for vision control.
"""

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
import argparse
import time
import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R

def parse_args():
    parser = argparse.ArgumentParser(description="NeRF Renderer for Camera Poses")
    parser.add_argument("--track_name", type=str, default="circle",
                        help="give track name")

    return parser.parse_args()


class NerfRenderer:
    """
    A class for rendering NeRF scenes based on camera poses.

    Attributes:
        config_path (str): Path to the configuration file.
        width (int): Image width.
        height (int): Image height.
        fx (float): Focal length in the x-axis.
        fy (float): Focal length in the y-axis.
        distortion_params (np.ndarray): Camera distortion parameters.
        camera_type (CameraType): Type of camera.
        metadata (any): Additional metadata.
        cx (float): Principal point x-coordinate.
        cy (float): Principal point y-coordinate.
        track (str): Track path.
    """

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
        cy = None,
        track = track_path
    ):

        """
        Initialize the NerfRenderer.

        Parameters:
            config_path (str): Path to the configuration file.
            width (int): Image width.
            height (int): Image height.
            fx (float): Focal length in the x-axis.
            fy (float): Focal length in the y-axis.
            distortion_params (np.ndarray): Camera distortion parameters. Defaults to None.
            camera_type (CameraType): Type of camera. Defaults to CameraType.PERSPECTIVE.
            metadata (any): Additional metadata. Defaults to None.
            cx (float): Principal point x-coordinate. Defaults to None.
            cy (float): Principal point y-coordinate. Defaults to None.
            track (str): Track path. Defaults to None.
        """

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
        self.nerfW = width
        self.nerfH = height
        self.distortion_params = distortion_params
        self.camera_type  = camera_type

        self.focal = self.fx

        self.metadata = metadata

        _, pipeline, _, step = eval_setup(
            self.config_path,
            eval_num_rays_per_chunk=640*480//4,
            test_mode='inference'
        )
        self.model = pipeline.model


        with open("./outputs/"+track_path+"/nerfacto/"+track_path+"/dataparser_transforms.json", 'r') as f:
            self.dp_trans_info = json.load(f)

    def render(self, cam_state:np.ndarray):
        """
        Render an image based on the given camera pose.

        Parameters:
            cam_state (np.ndarray): 4x4 camera pose matrix.

        Returns:
            np.ndarray: Rendered image.
        """

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
        s = time.time()
        with torch.no_grad():
            tmp = self.model.get_outputs_for_camera_ray_bundle(ray_bundle)
        print(f"NN Inference setup time: {round(time.time() - s, 2)}s")

        # Extract RGB image
        img = tmp['rgb']
        img =(colormaps.apply_colormap(image=img, colormap_options=colormaps.ColormapOptions())).cpu().numpy() # Apply colormap
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = (img * 255).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img


if __name__ == "__main__":

    args = parse_args()
    track_path = args.track_path
    width, height = 640, 480
    fov = 80
    fx = (width/2)/(np.tan(np.deg2rad(fov)/2))
    fy = (height/2)/(np.tan(np.deg2rad(fov)/2))

    renderer = NerfRenderer(
        '../outputs/'+track_path+'/nerfacto/'+track_path+'/config.yml',
        width = width,
        height = height,
        fx = 546.84164912, 
        fy = 547.57957461,
        cx = 349.18316327,
        cy = 215.54486004, 
        track = track_path
    )


    

    # TODO: replace w/ continuous trajectory / RRT Planner
    ## provide an arbitary [x, y, z, roll, pitch, yaw] to get fpv from that location
    ## You can also use the states of drone from ece484_state_controller.py to generate images along the path but adjust the yaw value along the path, Use it to create training data
    pose_list = [
        # [1.5, -0.15, 0.4, 0, 0, 3.14], # x, y, z, roll, pitch, yaw
        # [1.5, -0.15, 0.4, 0, 0, -1.57 + 1.57],
        # [-0.25, -1.7, 0.4, 0, 0,-3.14 + 1.57],
        # [-1.8, 0.05, 0.4, 0, 0, 1.57 + 1.57],
        # [-0.05, 1.6, 0.4, 0, 0, 0 + 1.57],

        [-2.1, 1.1, 0.85, 0, 0, -0.7]



    ]
    for idx, (x, y, z, roll, pitch, yaw) in enumerate(pose_list):



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
        print(f"Total Inference taken: {round(time.time() - s, 3)}s\n")  # Print total inference time
        
        plt.imshow(img)
        plt.show()

    
    
