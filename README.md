
<div align="center">
    <a href="https://ieeexplore.ieee.org/abstract/document/10715681" style="text-decoration:none;">
    <img src="https://img.shields.io/badge/Paper-T--IV-blue"/></a>
</div>

<div align="center">
<img src="docs/logo.png" align="right" width="30%">
</div>

# Radar4Motion: IMU-Free 4D Radar Odometry with Robust Dynamic Filtering and RCS-Weighted Matching

Radar4Motion is a robust odometry method that utilizes Doppler and RCS information from the 4D imaging radar's point cloud, even in the presence of noisy and sparse point cloud data.

<div align="center">
<p float="center">
<img src="./docs/vod03.gif" width="600"/>
<br />
<b>View-of-Delft Dataset Seq 03</b>
</p>
</div>

<div align="center">
<p float="center">
<img src="./docs/vod17.gif" width="600"/>
<br />
<b>View-of-Delft Dataset Seq 17</b>
</p>
</div>

- The above `gif` shows **ONLY** odometry-based mapping results.
    - *NO inertial sensor, NO GNSS sensor, NO loop-closure*
    - **Only Single front-view 4D Imaging Radar!**

## Prerequisites

To run this project, you need:
- ROS (Robot Operating System), tested with ROS Noetic
- Eigen3, for matrix and vector operations
- PCL (Point Cloud Library), for handling radar point cloud data
- nlohmann_json, for JSON parsing

## Installation

### Clone the Repository

```sh
cd ~/catkin_ws/src
git clone https://github.com/ailab-hanyang/Radar4Motion.git
```

### Build the Project

Using catkin, set up your workspace and build the project:

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Using the [View-of-Delft (VoD) Dataset](https://github.com/tudelft-iv/view-of-delft-dataset)
If you want to evaluate Radar4Motion with the View-of-Delft (VoD) dataset, please do the following:

1. Download or place the VoD dataset in your preferred location.
2. Set VoD datset path ```str_vod_dataset_base_path``` in [`/launch/radar4motion_offline.launch`](./launch/radar4motion_offline.launch).
    - OR, create a symbolic link inside the dataset/ folder so the code can locate VoD data under dataset/view_of_delft_PUBLIC. 
    - Set ```<param name="str_vod_dataset_base_path" value="$(find radar4motion)/dataset/"/>```
    - For example:
        ```sh
        cd ~/catkin_ws/src/Radar4Motion
        mkdir -p dataset
        ln -s /path/to/view_of_delft_PUBLIC dataset/
        ```

        Folder tree:
        ```sh
        Radar4Motion/
        ├── src/
        │   ├── .cpp files..
        └── dataset/
            └── clips/
            └── view_of_delft_PUBLIC/  -> symlink to /path/to/view_of_delft_PUBLIC
                ├── radar/
                │   ├── training/
                │   │   ├── velodyne/
                │   │   │   ├── (bin files)
                │   │   ├── pose/
                │   │   │   ├── (label files)
        ```

3. Create a test folder for evaluation (option)
    ```sh
    cd ~/catkin_ws/src/Radar4Motion
    mkdir -p test
    ```

## Usage
To start the radar odometry processing, launch the provided ROS launch files:
(**Deactivate any active conda environment.**)
- View-of-delft dataset
    ```sh
    cd ~/catkin_ws/src
    source devel/setup.bash
    roslaunch radar4motion radar4motion_offline.launch
    ```
- ROS topic
    ```sh
    cd ~/catkin_ws/src
    source devel/setup.bash
    roslaunch radar4motion radar4motion_online.launch
    ```
- **Note**: The *accumulated scans-to-submap matching* algorithm requires **precise sensor-vehicle calibration**.  
    Therefore, you must accurately set the following parameters in the `./config/radar_point_cloud_odometry.ini` file according to your environment:

    ```sh
    m_d_radar_calib_x_m = 3.5
    m_d_radar_calib_y_m = 0.0
    m_d_radar_calib_z_m = 0.0
    m_d_radar_calib_roll_deg = 1.0
    m_d_radar_calib_pitch_deg = -0.568
    m_d_radar_calib_yaw_deg = 0.43
    ```
    
    - In particular, for the View-of-Delft (VOD) dataset, exact sensor-vehicle calibration values are not provided by the dataset.
    - Hence, we have set approximate values based on the sensor mounting position and vehicle images. These values will be updated in the future through our ongoing *radar4selfcalibration* research, which will estimate these parameters more accurately.

### Configurations
Detailed descriptions of parameters can be found in [PARAMETERS](./docs/PARAMETERS.md).
1. ROS & File path configuration
    - File: [`/launch/radar4motion_[offline/online].launch`](./launch/radar4motion_offline.launch)
2. Radar ego motion estimation (ref: [REVE](https://github.com/christopherdoer/reve))
    - File: [`/config/radar_ego_motion_estimation.ini`](./config/radar_ego_motion_estimation.ini)
3. Odometry
    - File: [`/config/radar_point_cloud_odometry.ini`](./config/radar_point_cloud_odometry.ini)

## TODO
- [ ] Update voxel struct (frame-scan unit)
- [ ] Update VoD calibration params with Radar4SelfCalibration

## License

This project is licensed under the Apache-2.0 License - see the [LICENSE.md](LICENSE) file for details.

## Authors

- **Soyeong Kim** - soyeongkim@hanyang.ac.kr
- **Jiwon Seok** - jiwonseok@hanyang.ac.kr

## Citation
If you find this work useful, please cite the following paper:
```bibtex
@article{kim2024radar4motion,
title={Radar4Motion: IMU-Free 4D Radar Odometry with Robust Dynamic Filtering and RCS-Weighted Matching},
author={Kim, Soyeong and Seok, Jiwon and Lee, Jaehwan and Jo, Kichun},
journal={IEEE Transactions on Intelligent Vehicles},
year={2024},
publisher={IEEE}
}
```

## Acknowledgement
We would like to express our gratitude to all the contributors and resources that made this research possible.
- In the development of this package, we refer to [KISS-ICP](https://github.com/PRBonn/kiss-icp) and [REVE](https://github.com/christopherdoer/reve) for source codes.
- Dataset: [*View-of-Delft (VoD)*](https://github.com/tudelft-iv/view-of-delft-dataset)
- Evaluation: [evo](https://github.com/MichaelGrupp/evo) package for odometry evaluation