
# Contrast Maximization for Event-based Cameras
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Project Overview

The Contrast Maximization method plays a pivotal role in event-based vision applications, offering innovative solutions for challenges like motion, depth, and optical flow estimation. Despite its importance, there hasn't been an accessible open-source implementation available online.

To empower the research community working with event-based cameras, I've created this repository featuring an efficient implementation of the Contrast Maximization framework.

<div align="center">
    <img src="./output/rotation3d.gif" width="750">
    <div style="color: gray; font-size: 10px;">
        Contrast Maximization for Estimating Camera 3D Rotation
    </div>
</div>

&nbsp;

<div align="center">
    <img src="./output/translation2d.gif" width="750">
    <div style="color: gray; font-size: 10px;">
        Contrast Maximization for Estimating 2D Translation
    </div>
</div>

&nbsp;

## Key Features

- **High Efficiency**: Utilizing the Ceres solver for optimization and parallel computation, this implementation delivers a highly efficient solution, achieving near real-time performance under certain conditions.

- **Flexible Applications**: The repository is built on ROS, making it straightforward to compile, run, and integrate into other projects. We also provide test data to facilitate easy testing.

## Citation

This project is part of our research paper. If you find this repository useful for your academic projects, please consider citing our paper and the original Contrast Maximization paper.

```bib
@article{xing2023target,
  title={Target-free Extrinsic Calibration of Event-LiDAR Dyad using Edge Correspondences},
  author={Xing, Wanli and Lin, Shijie and Yang, Lei and Pan, Jia},
  journal={IEEE Robotics and Automation Letters},
  year={2023},
  publisher={IEEE}
}
```

## Usage Guide

Here's how you can start using this repository for your research or projects:

### 1. **Clone the Repository**

Begin by cloning the repository into your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/FORREST1901/contrast-maximization-for-event-cameras.git
```

### 2. **Install Dependencies**

This project relies primarily on the following packages:

- **Ceres Solver**: Follow the [installation guide](http://ceres-solver.org/installation.html).
- **dv-ros**: Install from the [inivation/dv-ros GitLab repository](https://gitlab.com/inivation/dv/dv-ros).

### 3. **Build the Project**

After the dependencies are installed, build the ROS workspace:

```bash
cd ~/catkin_ws
catkin_make
```

### 4. **Download Test Data**

Download the dataset via the [OneDrive link](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/wlxing_connect_hku_hk/ErQixBQhEjxIlxc4azGQjsgB98izRoB0s5iW4XAY2wtWhw). Place the data files in the `/data` folder within the project directory:

```bash
mkdir -p ~/catkin_ws/src/contrast-maximization-for-event-cameras/data
mv <downloaded-data> ~/catkin_ws/src/contrast-maximization-for-event-cameras/data
```

### 5. **Launch and Run**

Use the provided ROS launch files to start the framework:

- For 2D translation:
```bash
roslaunch contrast_maximization cm_translation_2d.launch
```

- For 3D rotation:
```bash
roslaunch contrast_maximization cm_rotation_3d.launch
```

## Contact

For further questions or support, please reach out to us at [wlxing@connect.hku.hk].

Thank you for your interest in our research!
