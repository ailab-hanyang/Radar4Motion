# Parameter Explanations
## [`/launch/radar4motion.launch`](/launch/radar4motion.launch)
- b_flag_run_vod_dataset (bool)
    - If set to ```true```, the node will read point clouds from the View-of-Delft dataset located in str_vod_dataset_base_path.
    - If set to ```false```, the node will switch to online mode, subscribing to a live topic (e.g., /afi910_cloud_node/cloud) of type sensor_msgs/PointCloud2.

- str_vod_dataset_base_path (string)
    - Specifies the path where the VoD dataset is located.
    - If you have symbolically linked the dataset to a local dataset/ folder (e.g., ln -s /path/to/view_of_delft_PUBLIC dataset/view_of_delft_PUBLIC), set this to $(find radar4motion)/dataset/.

- sequence_num (int)
    - Indicates which VoD dataset sequence to load for offline processing.
    - The actual sequence metadata (clip information) is stored in text files (.txt) under the clips/ folder.
    - Note: We gratefully utilized the .txt files provided by [CMFlow](https://github.com/Toytiny/CMFlow)). However, slight modifications were made as the sequence numbers in the provided files differ from those referenced in major papers. Please ensure the correct mapping when using the sequences.

- i_vod_frame_rate (int)
    - Defines a virtual frame rate (in Hz) for sequentially reading and publishing VoD data.
    - Useful for controlling how fast the offline dataset is “played back” to the odometry node.

- vod_eval_folder_path (string)
    - When this is set, the system will store evaluation results (e.g., estimated poses) in the specified directory.
    - These results can subsequently be analyzed using tools like evo for odometry evaluation.

- s_input_points_ (string)
    - Online mode parameter: If b_flag_run_vod_dataset = false, the node will subscribe to this topic to receive live radar/lidar point clouds.
    - Default: /afi910_cloud_node/cloud.

- str_origin_ & str_base_link_ (string)
    - Used to define the TF frames in the transformation tree.
    - str_origin_ is often set to "Map", while str_base_link_ is the vehicle or sensor reference frame (e.g., "radar").

- str_rcs_field_name_, str_power_field_name_ (string)
    - Defines the field names for RCS or power in the incoming point cloud.
    - Adjust if your point cloud messages use different field names.

- b_accum_whole_point_clouds_ (bool)
    - If true, the code will accumulate the entire point cloud over time for visualization or debugging purposes.

- d_map_voxel_leaf_size_ (double)
    - Sets the voxel size for downsampling the accumulated point cloud.
    - Larger values yield more aggressive downsampling.

- min_static_points_num (int)
    - Minimum number of “static” points required by the radar ego-motion module.
    - If fewer points are detected as static, a warning will be issued, and the algorithm may rely on a different motion model.

## [`/config/radar_point_cloud_odometry.ini`](/config/radar_point_cloud_odometry.ini)
- m_i_submap_scan_num
    - Specifies how many scans are retained to build a local “submap.” For example, if m_i_submap_scan_num = 5, the odometry algorithm will accumulate the most recent 5 scans and use them for matching.

- m_i_m_scan_num
    - Determines how many consecutive scans (“M-scan accumulation”) are grouped together for a single matching step. Increasing this value can improve matching robustness but may also increase computation time.

- m_d_radar_ego_estimation_vel_x_sigma_threshold_ms
    - Defines the maximum allowed uncertainty (standard deviation) in the radar-based ego-motion estimate along the X-axis (in m/s). If the radar’s estimated velocity uncertainty is larger than this threshold, the system will revert to using an alternative prediction (e.g., KISS-ICP) for the X-axis.

- m_d_radar_ego_estimation_vel_y_sigma_threshold_ms
    - Same concept as above, but for the Y-axis velocity uncertainty threshold.

- m_d_radar_ego_estimation_vel_z_sigma_threshold_ms
    - Same concept as above, but for the Z-axis velocity uncertainty threshold.

- m_i_min_input_points_num
    - Defines the minimum number of points required to perform the odometry update. If a scan contains fewer points than this value, the algorithm will only use a prediction step without matching.

- Radar Calibration Parameters (m_d_radar_calib_x_m, m_d_radar_calib_y_m, m_d_radar_calib_z_m,
    - m_d_radar_calib_roll_deg, m_d_radar_calib_pitch_deg, m_d_radar_calib_yaw_deg)
        - Provide the translation (in meters) and rotation (in degrees) offsets to transform from the radar frame to the vehicle frame. These values are used to construct an Eigen::Affine3d representing the radar’s mounting position and orientation.