# Contrast Maximization for Event-based Cameras
[![License: GNU GENERAL PUBLIC LICENSE
Version 2, June 1991](https://img.shields.io/badge/License-GPLv2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.html)

## Project Introduction

The Contrast Maximization method is a crucial and highly useful approach for event-based vision applications, providing solutions for challenges like motion, depth, and optical flow estimation. Despite its significance, there hasn't been an accessible open-source implementation available online. 

To support and empower the research community focused on event cameras, I've created this repository with an efficient implementation of the Contrast Maximization framework.

<div align="center">
    <div align="center">
        <img src="./output/rotation3d.gif" width="750">
    </div>
    <div style="color: gray; font-size: 10px;">
        Contrast Maximization for Estimating Camera 3D Rotation
    </div>
</div>
&nbsp;

<div align="center">
    <div align="center">
        <img src="./output/translation2d.gif" width="750">
    </div>
    <div style="color: gray; font-size: 10px;">
        Contrast Maximization for Estimating 2D Translation
    </div>
</div>
&nbsp;

## Key Features:

- **High Efficiency**: The project leverages the Ceres solver for optimization and parallel computation to deliver a highly efficient solution, achieving near real-time performance under certain conditions.

- **Flexible Applications**: The repository uses ROS, making it very easy to compile, run, and integrate into other projects. Additionally, we provide test data to facilitate one-click testing.

## Citation

The project is part of our paper, if you use this repository for academic projects, please consider citing our paper and the original Contrast Maximization paper. 

```bib
@article{xing2023target,
  title={Target-free Extrinsic Calibration of Event-LiDAR Dyad using Edge Correspondences},
  author={Xing, Wanli and Lin, Shijie and Yang, Lei and Pan, Jia},
  journal={IEEE Robotics and Automation Letters},
  year={2023},
  publisher={IEEE}
}
```

## Usage

To get started, follow the step-by-step instructions below:

1. **Dataset Download**: 
   - Begin by downloading our datasets from our provided [OneDrive link](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/wlxing_connect_hku_hk/ErQixBQhEjxIlxc4azGQjsgB98izRoB0s5iW4XAY2wtWhw). 
   - Additionally, if you wish to simulate more data yourself, you can leverage the configurations we provide and use the ESIM tool, available at [ESIM's GitHub Repository](https://github.com/uzh-rpg/rpg_esim).

2. **Environment Setup**:
   - Navigate to the `./config` directory where you can find the `ssac.yaml` file.
   - Use this yaml file to create a conda environment by running:
     ```
     conda env create -f ssac.yaml
     ```
   - Activate the newly created environment:
     ```
     conda activate ssac
     ```

3. **MATLAB Engine API Setup**:
    - Locate your MATLAB root directory. Once located, navigate to the external engines python directory by running:
        ```
        cd "matlabroot/extern/engines/python"
        ```
    - Inside this directory, set up the MATLAB Engine API by executing:
        ```
        python setup.py install
        ```

4. **Running the Code**:
   - Navigate to the `./src` directory.
   - To execute the main script, run:
     ```
     python esim_ssac.py
     ```
   - The configuration settings within the script can be easily modified to suit your needs or to experiment with different parameters.

## Usage

To use this repository for your own research or projects, follow these steps:

1. **Clone the Repository:**

   Begin by cloning the repository into your ROS workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/FORREST1901/contrast-maximization-for-event-cameras.git
   ```

2. **Install Dependencies:**

   This project primarily relies on the following packages:
    - **Ceres Solver**: Follow [Ceres-Solver's installation guide](http://ceres-solver.org/installation.html).
    - **dv-ros**: Install from the [inivation/dv-ros](https://gitlab.com/inivation/dv/dv-ros) GitLab repository.

3. **Build the Project:**

   Once dependencies are in place, build the ROS workspace:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

4. **Download Test Data:**

   Download our provided dataset via [OneDrive link](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/wlxing_connect_hku_hk/ErQixBQhEjxIlxc4azGQjsgB98izRoB0s5iW4XAY2wtWhw). Place the data files in the `/data` folder within the project directory:

   ```bash
   mkdir -p ~/catkin_ws/src/contrast-maximization-for-event-cameras/data
   mv <downloaded-data> ~/catkin_ws/src/contrast-maximization-for-event-cameras/data
   ```

5. **Launch and Run:**

   Run the framework using the provided ROS launch files:

    - For translation in 2D:
   ```bash
   roslaunch contrast_maximization cm_translation_2d.launch
   ```

    - For rotation in 3D:
   ```bash
   roslaunch contrast_maximization cm_rotation_3d.launch
   ```

## Contact

For further inquiries or questions, please contact us at [wlxing@connect.hku.hk].

Thank you for your interest in our research.

