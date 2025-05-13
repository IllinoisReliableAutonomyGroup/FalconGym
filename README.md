# FalconWing

Short Description: Open-source Fixed-wing Research Platform with realistic simulator and dynamcis.

Contact Info: Yan Miao (yanmiao2@illinois.edu)

For more details, please check our [paper](https://arxiv.org/abs/2505.01383) and [video](https://www.youtube.com/watch?v=nB9Yot0wU6ohttps://www.youtube.com/watch?v=nB9Yot0wU6o).

To cite our work, you can use 
```
@misc{miao2025falconwingopensourceplatformultralight,
      title={FalconWing: An Open-Source Platform for Ultra-Light Fixed-Wing Aircraft Research}, 
      author={Yan Miao and Will Shen and Hang Cui and Sayan Mitra},
      year={2025},
      eprint={2505.01383},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2505.01383}, 
}
```

## Installation

1. Follow tutorial on [NeRFStudio](https://docs.nerf.studio/quickstart/installation.html) to install both Conda and NeRFStudio
2. Clone this repository using Git and switch to Stock_Pavilion branch:
```bash
conda activate nerfstudio
git clone https://github.com/IllinoisReliableAutonomyGroup/FalconGym.git
cd FalconGym
git checkout Stock_Pavilion
```
3. Download the Stock Pavilion configuration files from [Yan's Google Drive](https://drive.google.com/drive/folders/1jTHsK4VBWPVTSzpj2DqrC5h4JKxUhX1u?usp=sharing) and place in specific folder hierachy
    - FalconGym (Stock_Pavilion branch)/
        - *.py (scripts from this github repo)
        - 0413_SP_analog_2 (analog version Stock Pavilion)
        - 0413_SP_iPhone (digital version Stock Pavilion)
        - outputs (ckpt files)
4. To visually inspect the track, you could run
``` bash
source ~/miniconda3/bin/activate
conda activate nerfstudio
ns-viewer --load-config outputs/0413_SP_iPhone/splatfacto/2025-04-23_225512/config.yml
```
Then open the web GUI using the link printed in the terminal

## Script Explanation

1. `generate_image.py`: you can render single image from any position in either Analog-image-based GSplat or Digital-image-based GSplat.
    - Check my commented model path to switch
2. `closed_loop.py`: set initial condition and target point, it runs a closed loop tracking. 
    - Dynamics and tracking controller are in `fixed_wing_physics.py`
    - Output is a txt file (`fixed_wing_traj.txt`) logging all states and control.
3. `generate_trajectory.py`: read the states from above txt file and render the corresponding images in `/visualize_trajectory_testing/*.png`.
4. `generate-video.py`: concatenate above images into a `.mp4` video.
