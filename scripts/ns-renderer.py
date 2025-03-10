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
            eval_num_rays_per_chunk=640*480//4,
            test_mode='inference'
        )
        self.model = pipeline.model 

        # 
        # outputs/1125_multiple_gates/nerfacto/2024-11-25_193859
        with open("./outputs/circle/nerfacto/circle/dataparser_transforms.json", 'r') as f:
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
        s = time.time()
        with torch.no_grad():
            tmp = self.model.get_outputs_for_camera_ray_bundle(ray_bundle)
        print(f"NN Inference setup time: {round(time.time() - s, 2)}s")

        img = tmp['rgb']
        img =(colormaps.apply_colormap(image=img, colormap_options=colormaps.ColormapOptions())).cpu().numpy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = (img * 255).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img

class SplatRenderer:
    def __init__(self,
            config_path: str,    
            width: int,
            height: int,
            # fov: float,
            fx: float, 
            fy: float, 
            distortion_params: np.ndarray = None,
            camera_type = CameraType.PERSPECTIVE,
            metadata = None,
            cx = None,
            cy = None,
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

        with open("./outputs/1125_multiple_gates/splatfacto/2024-11-25_194502/dataparser_transforms.json", 'r') as f:
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

        output = self.model.get_outputs(camera)
        img = output['rgb'].detach().cpu().numpy()
        with torch.no_grad():
            # tmp = self.model.get_outputs_for_camera_ray_bundle(ray_bundle)
            tmp = self.model.get_outputs_for_camera(camera)

        img = tmp['rgb']
        img =(colormaps.apply_colormap(image=img, colormap_options=colormaps.ColormapOptions())).cpu().numpy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = (img * 255).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img
    
class GazeboSplatRenderer(SplatRenderer):
    def __init__(
        self, 
        config_path: str, 
        width: int, 
        height: int, 
        fx: float, 
        fy: float, 
        distortion_params: np.ndarray = None,
        camera_type = CameraType.PERSPECTIVE
    ):
        super().__init__(
            config_path,
            width,
            height,
            fx,
            fy,
            distortion_params,
            camera_type
        )

        script_dir = os.path.dirname(os.path.realpath(__file__))
        config_path = os.path.join(script_dir,'../outputs/gazebo4_resampled3_dataset/splatfacto/2024-08-19_160517/config.yml')
        config_path = os.path.normpath(config_path)
        
        self.dp_trans_info = dpDict[config_path]

    def render(self, point: np.ndarray):
        # Set aircraft to pos
        camera_pose = np.zeros((4,4))
        camera_pose[3,3] = 1
        camera_pose[:3,:3] = Rotation.from_euler('xyz',[point[5],point[4],point[3]]).as_matrix()
        camera_pose[:3,3] = point[:3]

        # Convert camera pose to what's stated in transforms_orig.json
        tmp = Rotation.from_euler('zyx',[-np.pi/2,np.pi/2,0]).as_matrix()
        mat = camera_pose[:3,:3]@tmp 
        camera_pose[:3,:3] = mat 
        
        # Convert camera pose to Colmap frame in transforms.json
        camera_pose[0:3,1:3] *= -1
        camera_pose = camera_pose[np.array([0,2,1,3]),:]
        camera_pose[2,:] *= -1 

        transform = np.array(self.dp_trans_info['transform'])
        scale_factor = self.dp_trans_info['scale']
        camera_pose = transform@camera_pose 
        camera_pose[:3,3] *= scale_factor
        camera_pose = camera_pose[:3,:]

        image = super().render(camera_pose) 
        
        return img 


if __name__ == "__main__":
    width, height = 640, 480
    fov = 80
    fx = (width/2)/(np.tan(np.deg2rad(fov)/2))
    fy = (height/2)/(np.tan(np.deg2rad(fov)/2))

    renderer = NerfRenderer(
        #  outputs/1125_multiple_gates/nerfacto/2024-11-25_193859
        '../outputs/circle/nerfacto/circle/config.yml',
        width = width,
        height = height,
        fx = 546.84164912, 
        fy = 547.57957461,
        # fx = fx,
        # fy = fy,
        cx = 349.18316327,
        cy = 215.54486004
    )

    # renderer = SplatRenderer(
    #     '../outputs/1125_multiple_gates/splatfacto/2024-11-25_194502/config.yml',
    #     width = 640,
    #     height = 480,
    #     fx = 546.84164912, 
    #     fy = 547.57957461,
    #     cx = 349.18316327,
    #     cy = 215.54486004
    # )

    camera_pose = np.array([ 
                # [-0.41614684,  0.90929743,  0.,          1.        ],
                # [-0.90929743, -0.41614684, -0.,          1.        ],
                # [-0.        ,  0.        ,  1.,          1.        ],
                # [ 0.        ,  0.        ,  0.,          1.        ],

                                [
                    -0.9949476107823163,
                    -0.002555492995584587,
                    -0.10036294761569253,
                    -0.2140848459824146
                ],
                [
                    -0.0006767843752171741,
                    0.9998239979763963,
                    -0.018748734181405338,
                    1.0769276876074263
                ],
                [
                    0.10039319579269401,
                    -0.01858608420418495,
                    -0.9947742275071692,
                    -1.5625590621154097
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    1.0
                ]
                
    ])

    import numpy as np
    from scipy.spatial.transform import Rotation as R

    # TODO: replace w/ continuous trajectory / RRT Planner
    pose_list = [
        # [1.5, -0.15, 0.4, 0, 0, 3.14], # x, y, z, roll, pitch, yaw, gate_flag
        # [1.5, -0.15, 0.4, 0, 0, -1.57 + 1.57],
        # [-0.25, -1.7, 0.4, 0, 0,-3.14 + 1.57],
        # [-1.8, 0.05, 0.4, 0, 0, 1.57 + 1.57],
        # [-0.05, 1.6, 0.4, 0, 0, 0 + 1.57],

        [2, 2, 0.4, 0, 0, -2.5]

        # [1, 2, 1, 0, 0, -2],
        # [0, 2, 0.8, 0, 0, -1.57],
        # [-1, 2, 0.8, 0, 0, -1],

        # # [0, 2.5, 0.85, 0, 0, -1.57]
        # [0, 0, 0.85, 0, 0, -2.5]

    ]
    for idx, (x, y, z, roll, pitch, yaw) in enumerate(pose_list):

        # pitch = np.radians(0)  # example pitch in radians
        # yaw = -.5    # example yaw in radians
        # roll = np.radians(0)   # example roll in radians
        # x, y, z = -1, 1, 0.5    # example translation coordinates

        # Step 1: Create the rotation matrix from pitch, yaw, roll
        rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)  # 'xyz' assumes roll, pitch, yaw order
        # x = 0.1
        # y*=2
        # rotation = R.from_euler('xyz', [0,0,-1.57], degrees=False)  # 'xyz' assumes roll, pitch, yaw order
        rotation_matrix = rotation.as_matrix()  # Converts to a 3x3 rotation matrix

        # Step 2: Create the 4x4 transformation matrix
        camera_pose = np.eye(4)  # Start with an identity matrix
        camera_pose[:3, :3] = rotation_matrix  # Set the top-left 3x3 to the rotation matrix
        camera_pose[:3, 3] = [x, y, z]  # Set the translation components

        s = time.time()
        img = renderer.render(camera_pose) # RGB
        cv_img = img[:, :, ::-1]
        print(f"Total Inference taken: {round(time.time() - s, 3)}s\n")
        
        plt.imshow(img)
        plt.show()
        # break
        # cv2.imwrite(f"./scripts/output/{idx:04d}.png", cv_img)
