# FalconGym

### Contact Info: Yan Miao (yanmiao2@illinois.edu)

This repository provides a photorealistic simulation environment (FalconGym), that consists of 3 tracks, namely circle, U-turn and lemniscate (Figure-8). Each track consists of 4 gates arranged in different shapes.

This repository is intended for students working with FalconGym, specifically in ECE 484, to develop and evaluate drone control policies in a photorealistic simulation environment.

For more details on FalconGym, please refere to our [paper](https://arxiv.org/abs/2503.02198) and [video](https://www.youtube.com/watch?v=TuTugNa39xs). The following is the demo video for circle track using a vision-based controller [demo video](https://uillinoisedu-my.sharepoint.com/:v:/g/personal/mkg7_illinois_edu/Ef8JGuti9Q9MkdXbGLL4g2QBY8AK-GwkcmIff8aKlB7lWA?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=kVB2Lo).

To cite our work, you can use 
```
@misc{miao2025zeroshotsimtorealvisualquadrotor,
      title={Zero-Shot Sim-to-Real Visual Quadrotor Control with Hard Constraints}, 
      author={Yan Miao and Will Shen and Sayan Mitra},
      year={2025},
      eprint={2503.02198},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2503.02198}, 
}
```
<!-- ## Dependencies
    - python >= 3.8
    - PyTorch 2.1.2
    - CUDA 11.8
    - numpy 1.24.4
    - OpenCV 4.11.0
    - SciPy 1.10.1
    - Plotly 6.0 -->

## Installation

1. Follow tutorial on [NeRFStudio](https://docs.nerf.studio/quickstart/installation.html) to install both Conda and NeRFStudio
2. Clone this repository using Git:
```bash
conda activate nerfstudio
git clone https://github.com/IllinoisReliableAutonomyGroup/FalconGym.git
cd FalconGym
pip install -r requirements.txt
```
3. Download the track configuration files from [Yan's Google Drive](https://drive.google.com/drive/folders/14IWE-GJ0t2qyS2GCGHhRHTpKWM0x6Jrh?usp=sharing) and place in specific folder hierachy
    - FalconGym/
        - scripts/ (from this github repo)
        - circle/
        - uturn/
        - lemniscate/
        - outputs/
4. To visually inspect the track, you could run
```
source ~/miniconda3/bin/activate
conda activate nerfstudio
ns-viewer --load-config outputs/circle/nerfacto/circle/config.yml
```
Then open the web GUI using the link printed in the terminal

NOTE: To visualize the lemniscate or U-turn track, update the command as follows
```
ns-viewer --load-config outputs/'TRACK'/nerfacto/'TRACK'/config.yml
```
Replace TRACK with either lemniscate or uturn as needed.

![](images/gates_image.png)


## Scripts Explanation
This section describes the scripts available in FalconGym. Each script has specific functionality for drone simulation, control, and evaluation.

### `drone_dynamics.py` - Double Integrator-Based Drone Dynamics  
- **Purpose**: Simulates drone dynamics  
- **Input**:  
    - State: `(x, y, z, vx, vy, vz, yaw)`  
    - Control: `(ax, ay, az, yaw_rate)`  
- **Output**: Next state  
- **Notes**:  Keep `dt = 0.05s`  
- **Example Usage**:  `python3 drone_dynamics.py`


### `ns-renderer.py`:  generates RGB images of the environment, useful for dataset creation.
- **Purpose**: Environment overview, Use this to get a 3d view of each environment, You can also create a training datset for gate detection.
- **Input**: camera pose (x, y, z, roll, pitch, yaw)
- **Output**: RGB image (640x480x3)
- **Notes**: Modify Track path accordingly
- **Example Usage**: `python3 scripts/ns_renderer.py --track_name TRACK` (note the file hierarchy, TRACK = circle, lemniscate, uturn)
    
### `ece484-gate-detection.py`: extracts gates from RGB images.
- **TODO**: Write gate detection algorithm here.
- **Purpose**: Detects gates, generates a segmented binary mask of the gate.
- **Input**: RGB (640x480x3)
- **Output**: Mask (640x480)
- **Example Usage**: `python3 ece484-gate-detection.py`
- **Notes**: The functions in this file will be used by `ece484-vision-controller.py` and `ece484-vision-closed-loop.py`.

### `ece484-state-controller.py`: computes control commands based on state estimates.
- **TODO**: Write state controller algorithm here. This script manages drone movements by controlling acceleration and yaw rate based on the current state of the drone.
- **Input**: state (x, y, z, vx, vy, vz, yaw) + gate poses
- **Output**: control (ax, ay, az, yaw_rate)
- **Notes**: The controller function in this file will be used by `ece484-state-closed-loop.py`.

### `ece484-vision-controller.py`:  computes control commands from first-person images.
- **TODO**: Write your vision controller algorithm here. This script manages drone movements by controlling acceleration and yaw rate based on the fpv image of the drone.
- **Input**: 
    - RGB Image (640x480x3)
    - Binary mask (640x480x3) (Taken from  `ece484-gate-detection.py`)
- **Output**: control (ax, ay, az, yaw_rate)

###  `ece484-state-closed-loop.py`
    
- **Purpose**: This script runs `ece484-state-controller.py` in closed loop. It simulates the drone's dynamics, and saves the trajectory to a text file.
- **Input**:
    - State: `(x, y, z, vx, vy, vz, yaw)` (Taken from `drone_dynamics.py`).
    - Control: `(ax, ay, az, yaw_rate)`  (Taken from `ece484-state-controller.py`).
- **Output**: A trajectory.txt file. Contains (x,y,z,Yaw)
- **Example Usage**: `python3 ece484-state-closed-loop.py --track-name TRACK`. (TRACK = Circle_Track, Uturn_Track, Lemniscate_Track)
- **Notes**: 
    - Use this trajectory txt file for evaluating your controller using `ece484_evaluate.py`.
    - No edits required, run this after finishing ece484_statecontroller.py. 

###  `ece484-vision-closed-loop.py`
- **Purpose**: This script runs `ece484-vision-controller.py` in closed loop. It uses gate detection algorithm from `ece484-gate-detection.py` and vision controller from `ece484-vision-controller.py`. 
- **Input**:
    - State: `(x, y, z, vx, vy, vz, yaw)` (Taken from `drone_dynamics.py`).
    - Control: `(ax, ay, az, yaw_rate)`  (Taken from `ece484-vision-controller.py`).
- **Output**:
    - An image folder.
    - A trajectory.txt file. contains (x,y,z,Yaw)
    - A mp4 video.
- **Example Usage**: `python3 ece484-vision-closed-loop.py --track-name TRACK`. (TRACK = Circle_Track, Uturn_Track, Lemniscate_Track)
- **Notes**: 
    - Use this trajectory txt file for evaluating your controller using `ece484_evaluate.py`.
    - No edits required, run this after finishing ece484_statecontroller.py. 

###  `ece484_evaluate.py`: computes metrics gate errors, time, and success rate.
 - **Purpose**: TO evaluate controllers performance in each track using metrics MGE, LT, SR and Trajectory Visualization. 
 - **Input**: Trajectory.txt file generated from `ece484-vision-closed-loop.py` or `ece484-state-closed-loop.py`.
 - **Output**:
    - metrics.json file.
    - plot of the trajectory.
- **Example Usage**: `python3 ece484_evaluate.py --track-name Circle_Track --trajectory-path circle_traj.txt --visflag True --metricsflag True` take track name and trajectory txt file as arguments flags are optional.

### `ece484_videogenerator.py`: Generates a video from sequentially named images.
 - **Purpose**: Used to generate video using images of the scene.
 - **Input**:
    - Folder containing PNG images.
 - **Output**: MP4 video file.
- **Example Usage**: `python3 ece484_videogenerator.py --input ./closed_loop/images --output ./track_vision_trajectory.mp4 --fps 20`.
- **Notes**
    - `--input`: Path to the image folder.
    - `--output`: Path to save the video file.
    - `--fps`: Frames per second (default: 20).


## Tasks for ECE 484 students
1. **State-Based Controller**
    - Implement `ece484_state_controller.py` to navigate 2 laps (8 gates) in each of the three tracks.
    - Use `gates_pos.txt` for gate locations.
    - Run `ece484_state_closed_loop.py` to generate a trajectory file.
    - Evaluate using `ece484_evaluate.py`, reporting:
      - **SR** (Success Rate): % of gates successfully crossed.
      - **MGE** (Mean Gate Error): Avg. distance from gate center.
      - **LP** (Lap Time): `0.05 * # frames`.
    - Benchmark: check below
2. **Gate Detection**
    - Implement `ece484-gate-detection.py`
    - Collect image dataset using ns-renderer.py
    - You should demonstrate at least have around 100 images of different gates in different tracks (obtained from sampling using `ns-renderer.py`) where you can do gate detection perfectly through visual inspection. 
    - Yan's benchmark, Check `gate-detect-Yan-example/`
3. **Localization / SLAM**
    - You can free-style create or modify anything, the goal is to build on Task 2 to achieve:
        - Input: RGB Image
        - Output: gate relative pose to camera
    - Reference: [GateNet](https://github.com/open-airlab/GateNet)
4. **Vision-Based Control**
    - Implement `ece484_vision_controller.py` using insights from Tasks 2 & 3.
    - Run `ece484_vision_closed_loop.py` to generate a trajectory file and images.
    - Use `ece484_videogenerator.py` to generate a video from the output images.
    - Evaluate using `ece484_evaluate.py`, reporting SR, MGE, and LP.


## Env Explanation

![Circle Track](images/tracks.png)



## Submission File & Yan's Benchmark

### State-based Controller Benchmark
|          |   SR    |   MGE(cm) |  LP(s)  |
| -------- | ------- | --------  | ------- |
| Circle   | 100%    |   2.47    |    11   |
|Lemniscate| 100%    |   5.11    |    15   |
| Uturn    | 100%    |   3.42    |    7    |

### Vision-based Controller Benchmark
|          |   SR    |   MGE(cm) |  LP(s)  |
| -------- | ------- | --------  | ------- |
| Circle   | 100%    |   6.25    |    11   |
|Lemniscate| 100%    |   5.13    |    15   |
| Uturn    | 100%    |   10.1    |    7    |

Please include the below information in your final submission.
1. TASK1: State Controller
    - **CIRCLE** Trajectory.txt, metrics.json.
    - **UTURN** Trajectory.txt, metrics.json.
    - **LEMNISCATE** Trajectory.txt, metrics.json.
2. TASK4: Vision COntroller
    - **CIRCLE** Trajectory.txt, metrics.json and MP4 Video file.
    - **UTURN** Trajectory.txt, metrics.json and MP4 Video file.
    - **LEMNISCATE** Trajectory.txt, metrics.json and MP4 Video file.

-**NOTE**:
    - Trajectory.txt generated from `ece484-state-closed-loop.py` or `ece484-vision-closed-loop.py`
    - metrics.json generated from `ece484_evaluate.py`.
    - MP4 Video file generated from `ece484_videogenerator.py`.

