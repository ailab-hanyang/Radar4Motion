/**
 * @copyright (c) AI LAB - Hanyang Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted
 * disclosure only.
 * @author soyeongkim@hanyang.ac.kr, jiwonseok@hanyang.ac.kr
 * @file odometry.cpp
 * @brief radar4motion odometry source code
 * @version 1.0
 * @date 03-09-2024
 * @bug No known bugs
 * @warning No warnings
 */

#include "odometry.hpp"

bool CRadar4MotionOdometry::RunOdometry(double time_s,
                                        std::vector<std::pair<Eigen::Vector3d, double>> i_point_rcs_pairs,
                                        Eigen::Vector3d &i_vec_ego_motion_3dof_vel,
                                        Eigen::Vector3d &i_vec_ego_motion_3dof_vel_sigma) {
    // Data validation check
    if (i_point_rcs_pairs.size() < 1) {
        std::cout << "[ODOMETRY] NO INPUT POINTS!" << std::endl;
        return false;
    }
    if (time_s < m_d_prev_timestamp) {
        std::cout << "[ODOMETRY] INVALID TIMESTAMP!" << std::endl;
        return false;
    }

    // Parse config in real-time
    if (ParseINIRegistration() == false) {
        std::cout << "[ODOMETRY] INI PARSING ERROR!" << std::endl;
        return false;
    }

    // get last pose
    const auto last_pose = !m_odom_poses.empty() ? m_odom_poses.back() : Sophus::SE3d();

    // Convert std::vector<std::pair<Eigen::Vector3d, double>> to std::vector<Eigen::Vector3d> & std::vector<double>
    std::vector<Eigen::Vector3d> i_point_cloud;
    std::vector<double> i_point_cloud_rcs;

    ConvertRCSPointPair2PointCloud(i_point_rcs_pairs, i_point_cloud, i_point_cloud_rcs);

    m_submap_point_clouds.push_back(i_point_cloud);
    m_submap_point_rcs.push_back(i_point_cloud_rcs);

    // get init tf
    Sophus::SE3d init_ego_motion_tf;

    // Predict ego motion
    Vector6d vec6d_init_ego_motion_estimation;
    PredictEgoMotion(i_vec_ego_motion_3dof_vel, i_vec_ego_motion_3dof_vel_sigma, time_s,
                     vec6d_init_ego_motion_estimation);

    init_ego_motion_tf = Sophus::SE3d::exp(vec6d_init_ego_motion_estimation);

    // If the number of input points is too small, only prediction is performed.
    if (i_point_rcs_pairs.size() < m_i_min_input_points_num) {
        std::cout << "[ODOMETRY] TOO SMALL POINTS NUM! ONLY PREDICTION!" << std::endl;

        const auto initial_guess = last_pose * init_ego_motion_tf;
        m_odom_poses.push_back(initial_guess);
        m_submap_poses.push_back(initial_guess);

        // convert to affine3d
        Eigen::Affine3d result_tf;
        GetXYZRPY(initial_guess, result_tf);

        m_kiss_icp_voxel_hash_local_map.Update(i_point_cloud, initial_guess);
        m_odom_local_map = m_kiss_icp_voxel_hash_local_map.Pointcloud();

        m_result_tf = result_tf;
        m_d_prev_timestamp = time_s;
        // return true;
        return false;
    }

    // Store the initial guess for M scan accumulation
    static std::deque<Sophus::SE3d> init_guesses;
    init_guesses.push_back(init_ego_motion_tf);

    while (init_guesses.size() > m_i_m_scan_num) init_guesses.pop_front();

    // Accumulate the point cloud with initial_guess
    std::vector<Eigen::Vector3d> M_scan_accumulated_point_cloud;
    std::vector<double> M_scan_accumulated_rcs;

    if (m_i_m_scan_num > 1) {
        GetMScanAccumulatedPointCloud(i_point_cloud, i_point_cloud_rcs, init_guesses, last_pose, time_s,
                                      M_scan_accumulated_point_cloud, M_scan_accumulated_rcs);
    }
    else {
        M_scan_accumulated_point_cloud = i_point_cloud;
        M_scan_accumulated_rcs = i_point_cloud_rcs;
    }

    const auto initial_guess = last_pose * init_ego_motion_tf;

    const auto initail_guess_deviation = last_pose.inverse() * initial_guess;
    const double sigma = GetAdaptiveThreshold();

    std::vector<Eigen::Vector3d> correspondence_src;
    std::vector<Eigen::Vector3d> correspondence_tgt;

    Sophus::SE3d radar_odom_result = m_kiss_icp_registration.AlignPointsToMapWithRCS(
            M_scan_accumulated_point_cloud, M_scan_accumulated_rcs, m_kiss_icp_voxel_hash_local_map, initial_guess,
            3.0 * sigma, // max correspondence distance
            sigma / 3.0  // kernel
    );

    m_vec_correspondence_src = correspondence_src;
    m_vec_correspondence_tgt = correspondence_tgt;

    const Sophus::SE3d model_deviation = initial_guess.inverse() * radar_odom_result;
    m_kiss_icp_adaptive_threshold.UpdateModelDeviation(model_deviation);

    // Convert to affine3d
    Eigen::Affine3d result_tf;

    GetXYZRPY(radar_odom_result, result_tf);
    m_kiss_icp_voxel_hash_local_map.Clear();

    m_submap_poses.push_back(radar_odom_result);

    // For generating submap
    while (m_submap_poses.size() > m_i_submap_scan_num) {
        m_submap_poses.erase(m_submap_poses.begin());
        m_submap_point_clouds.erase(m_submap_point_clouds.begin());
        m_submap_point_rcs.erase(m_submap_point_rcs.begin());
    }

    // TODO: modify kiss icp voxel hash map -> manage submap with scan unit
    for (size_t i = 0; i < m_submap_poses.size(); i++) {
        m_kiss_icp_voxel_hash_local_map.Update(m_submap_point_clouds[i], m_submap_poses[i]);
    }

    m_odom_poses.push_back(radar_odom_result);

    m_odom_local_map = m_kiss_icp_voxel_hash_local_map.Pointcloud();
    m_result_tf = result_tf;
    m_d_prev_timestamp = time_s;

    return true;
}

void CRadar4MotionOdometry::ConvertRCSPointPair2PointCloud(
        const std::vector<std::pair<Eigen::Vector3d, double>> &i_point_rcs_pairs,
        std::vector<Eigen::Vector3d> &i_point_cloud, std::vector<double> &i_point_cloud_rcs) {
    i_point_cloud.reserve(i_point_rcs_pairs.size());
    i_point_cloud_rcs.reserve(i_point_rcs_pairs.size());

    for (auto idx = 0; idx < i_point_rcs_pairs.size(); idx++) {
        i_point_cloud.push_back(i_point_rcs_pairs[idx].first);
        i_point_cloud_rcs.push_back(i_point_rcs_pairs[idx].second);
    }
}

void CRadar4MotionOdometry::GetMScanAccumulatedPointCloud(const std::vector<Eigen::Vector3d> &i_point_cloud,
                                                          const std::vector<double> &i_point_cloud_rcs,
                                                          const std::deque<Sophus::SE3d> &init_guesses,
                                                          const Sophus::SE3d &last_pose, double time_s,
                                                          std::vector<Eigen::Vector3d> &M_scan_accumulated_point_cloud,
                                                          std::vector<double> &M_scan_accumulated_rcs) {
    auto point_cloud_iter = m_submap_point_clouds.rbegin();
    auto pose_iter = init_guesses.rbegin();
    auto rcs_iter = m_submap_point_rcs.rbegin();

    Sophus::SE3d last_pose_from_current = Sophus::SE3d::exp(Vector6d::Zero());

    M_scan_accumulated_point_cloud.reserve(i_point_cloud.size() * 10);
    M_scan_accumulated_rcs.reserve(i_point_cloud.size() * 10);

    // Iterate through the last M scans
    for (int i = 0; i < init_guesses.size(); i++) {
        if (point_cloud_iter == m_submap_point_clouds.rend()) break;
        if (pose_iter == init_guesses.rend()) break;

        std::vector<Eigen::Vector3d> tformed_point_cloud;
        tformed_point_cloud.reserve(point_cloud_iter->size());
        for (int point_idx = 0; point_idx < point_cloud_iter->size(); point_idx++) {
            tformed_point_cloud.push_back(last_pose_from_current * (*point_cloud_iter)[point_idx]);
        }

        M_scan_accumulated_point_cloud.insert(M_scan_accumulated_point_cloud.end(), tformed_point_cloud.begin(),
                                              tformed_point_cloud.end());
        M_scan_accumulated_rcs.insert(M_scan_accumulated_rcs.end(), rcs_iter->begin(), rcs_iter->end());

        last_pose_from_current = last_pose_from_current * (*pose_iter).inverse();

        point_cloud_iter++;
        pose_iter++;
        rcs_iter++;
    }
}

void CRadar4MotionOdometry::GetXYZRPY(const Sophus::SE3d &i_se3d_tf, Eigen::Affine3d &o_eigen_tf) {
    // get translation and rotation
    Eigen::Vector3d t_current = i_se3d_tf.translation();
    Eigen::Quaterniond q_current = i_se3d_tf.unit_quaternion();

    tf::Quaternion tf_odom_quat(q_current.x(), q_current.y(), q_current.z(), q_current.w());
    tf::Matrix3x3 odom_rot(tf_odom_quat);
    double roll, pitch, yaw;
    odom_rot.getRPY(roll, pitch, yaw);

    // convert to affine3d
    pcl::getTransformation(t_current.x(), t_current.y(), t_current.z(), roll, pitch, yaw, o_eigen_tf);
}

void CRadar4MotionOdometry::GetScaledSE3(const Sophus::SE3d &i_se3d_tf, const double scale, Sophus::SE3d &o_se3d) {
    // get translation and rotation
    Eigen::Vector3d t_current = i_se3d_tf.translation();
    Eigen::Quaterniond q_current = i_se3d_tf.unit_quaternion();

    tf::Quaternion tf_odom_quat(q_current.x(), q_current.y(), q_current.z(), q_current.w());
    tf::Matrix3x3 odom_rot(tf_odom_quat);
    double roll, pitch, yaw;
    odom_rot.getRPY(roll, pitch, yaw);

    // convert to scaled se3
    Eigen::Affine3d o_eigen_tf;
    pcl::getTransformation(t_current.x(), t_current.y(), t_current.z() * scale, roll * scale, pitch * scale, yaw,
                           o_eigen_tf);
    Sophus::SE3d output_se3(o_eigen_tf.matrix());
    o_se3d = output_se3;
}

void CRadar4MotionOdometry::PredictEgoMotion(Eigen::Vector3d &i_vec_ego_motion_3dof_vel,
                                             Eigen::Vector3d &i_vec_ego_motion_3dof_vel_sigma, double i_curr_point_time,
                                             Vector6d &o_pred_motion_6dof) {
    double delta_time_sec = 0.0;
    if (m_d_prev_timestamp > FLT_MIN) {
        delta_time_sec = i_curr_point_time - m_d_prev_timestamp;
    }

    if (delta_time_sec < FLT_MIN) {
        o_pred_motion_6dof.setZero();
        std::cout << "[ODOMETRY] delta time is zero!" << std::endl;
        return;
    }

    // Rotate radar velocity to vehicle frmae with calibration information.
    Eigen::Vector3d radar_vel_in_vehicle = m_tf_radar_calib.rotation() * i_vec_ego_motion_3dof_vel;

    // Velocity = translation X Rotation;
    Eigen::Vector3d roll_normal_vector = Eigen::Vector3d::UnitX().cross(m_tf_radar_calib.translation());

    roll_normal_vector = roll_normal_vector / roll_normal_vector.norm();
    double roll_normal_distance =
            sqrt(pow(m_tf_radar_calib.translation().y(), 2.0) + pow(m_tf_radar_calib.translation().z(), 2.0));

    double roll_rate = 0.0;
    if (roll_normal_distance > FLT_MIN) roll_rate = radar_vel_in_vehicle.dot(roll_normal_vector) / roll_normal_distance;

    Eigen::Vector3d pitch_normal_vector = Eigen::Vector3d::UnitY().cross(m_tf_radar_calib.translation());
    pitch_normal_vector = pitch_normal_vector / pitch_normal_vector.norm();

    double pitch_normal_distance =
            sqrt(pow(m_tf_radar_calib.translation().x(), 2.0) + pow(m_tf_radar_calib.translation().z(), 2.0));

    double pitch_rate = 0.0;
    if (pitch_normal_distance > FLT_MIN)
        pitch_rate = radar_vel_in_vehicle.dot(pitch_normal_vector) / pitch_normal_distance;

    Eigen::Vector3d yaw_normal_vector = Eigen::Vector3d::UnitZ().cross(m_tf_radar_calib.translation());
    yaw_normal_vector = yaw_normal_vector / yaw_normal_vector.norm();

    double yaw_normal_distance =
            sqrt(pow(m_tf_radar_calib.translation().x(), 2.0) + pow(m_tf_radar_calib.translation().y(), 2.0));

    double yaw_rate = 0.0;
    if (yaw_normal_distance > FLT_MIN) yaw_rate = radar_vel_in_vehicle.dot(yaw_normal_vector) / yaw_normal_distance;

    Eigen::Vector3d radar_ang_vel_in_vehicle = {roll_rate, pitch_rate, yaw_rate};
    Eigen::Vector3d radar_ang_vel_in_radar = m_tf_radar_calib.rotation().transpose() * radar_ang_vel_in_vehicle;

    // backup kiss icp prediction module
    const auto kiss_icp_predicted_motion = GetPredictionModel();
    Eigen::Vector3d kiss_icp_translation = kiss_icp_predicted_motion.translation();
    Eigen::Quaterniond kiss_icp_quat = kiss_icp_predicted_motion.unit_quaternion();

    // Predict init pose using motion data
    double delta_x_m = 0.0, delta_y_m = 0.0, delta_z_m = 0.0;

    // Convert quaternion to roll, pitch, yaw
    double delta_roll_rad = 0.0, delta_pitch_rad = 0.0, delta_yaw_rad = 0.0;
    tf::Matrix3x3(tf::Quaternion(kiss_icp_quat.x(), kiss_icp_quat.y(), kiss_icp_quat.z(), kiss_icp_quat.w()))
            .getRPY(delta_roll_rad, delta_pitch_rad, delta_yaw_rad);

    // x
    if (i_vec_ego_motion_3dof_vel_sigma[0] < m_d_radar_ego_estimation_vel_x_sigma_threshold_ms) {
        delta_x_m = i_vec_ego_motion_3dof_vel[0] * delta_time_sec;
    }
    else {
        delta_x_m = kiss_icp_translation[0];

        std::cout << "\033[1;36m"
                  << "[ODOMETRY] Radar Ego Estimation X Sigma " << i_vec_ego_motion_3dof_vel_sigma[0]
                  << " is larger than " << m_d_radar_ego_estimation_vel_x_sigma_threshold_ms << "\033[0m" << std::endl;
    }

    // y
    if (i_vec_ego_motion_3dof_vel_sigma[1] < m_d_radar_ego_estimation_vel_y_sigma_threshold_ms) {
        delta_y_m = i_vec_ego_motion_3dof_vel[1] * delta_time_sec;
    }
    else {
        delta_y_m = kiss_icp_translation[1];

        std::cout << "\033[1;36m"
                  << "[ODOMETRY] Radar Ego Estimation Y Sigma " << i_vec_ego_motion_3dof_vel_sigma[1]
                  << " is larger than " << m_d_radar_ego_estimation_vel_y_sigma_threshold_ms << "\033[0m" << std::endl;
    }

    // z
    if (i_vec_ego_motion_3dof_vel_sigma[2] < m_d_radar_ego_estimation_vel_z_sigma_threshold_ms) {
        delta_z_m = delta_time_sec * i_vec_ego_motion_3dof_vel[2];
    }
    else {
        delta_z_m = kiss_icp_translation[2];

        std::cout << "\033[1;36m"
                  << "[ODOMETRY] Radar Ego Estimation Z Sigma " << i_vec_ego_motion_3dof_vel_sigma[2]
                  << " is larger than " << m_d_radar_ego_estimation_vel_z_sigma_threshold_ms << "\033[0m" << std::endl;
    }

    // roll, pitch, yaw
    if (i_vec_ego_motion_3dof_vel_sigma[0] < m_d_radar_ego_estimation_vel_x_sigma_threshold_ms &&
        i_vec_ego_motion_3dof_vel_sigma[1] < m_d_radar_ego_estimation_vel_y_sigma_threshold_ms) {
        delta_roll_rad = radar_ang_vel_in_radar[0] * delta_time_sec;
        delta_pitch_rad = radar_ang_vel_in_radar[1] * delta_time_sec;
        delta_yaw_rad = radar_ang_vel_in_radar[2] * delta_time_sec;
    }

    o_pred_motion_6dof << delta_x_m, delta_y_m, delta_z_m, delta_roll_rad, delta_pitch_rad, delta_yaw_rad;

    // For visualization & evaluation
    m_vec6d_init_ego_motion_estimation = o_pred_motion_6dof;
}

Sophus::SE3d CRadar4MotionOdometry::GetPredictionModel() const {
    Sophus::SE3d pred = Sophus::SE3d();
    const size_t N = m_odom_poses.size();

    if (N < 2) return pred;
    return m_odom_poses[N - 2].inverse() * m_odom_poses[N - 1];
}

double CRadar4MotionOdometry::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        return 2.0;
    }
    return m_kiss_icp_adaptive_threshold.ComputeThreshold();
}

bool CRadar4MotionOdometry::HasMoved() {
    if (m_odom_poses.empty()) return false;
    const double motion = (m_odom_poses.front().inverse() * m_odom_poses.back()).translation().norm();
    return motion > 5.0 * 0.1;
}

// Configuration
bool CRadar4MotionOdometry::Init(std::string ini_path) {
    if (registration_ini_handler.Init(ini_path.c_str()) == false) return false;
    if (ParseINIRegistration() == false) return false;

    m_d_prev_timestamp = 0.0;
    m_kiss_icp_voxel_hash_local_map.Clear();
    m_odom_poses.clear();
    m_odom_local_map.clear();
    m_submap_poses.clear();
    m_submap_point_clouds.clear();
    m_submap_point_rcs.clear();

    return true;
}

bool CRadar4MotionOdometry::Init() {
    m_d_prev_timestamp = 0.0;
    m_kiss_icp_voxel_hash_local_map.Clear();
    m_odom_poses.clear();
    m_odom_local_map.clear();
    m_submap_poses.clear();
    m_submap_point_clouds.clear();
    m_submap_point_rcs.clear();

    return true;
}

bool CRadar4MotionOdometry::ParseINIRegistration() {
    if (registration_ini_handler.IsFileUpdated() == false) {
        return true;
    }
    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_i_submap_scan_num", m_i_submap_scan_num) != true)
        return false;
    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_d_radar_ego_estimation_vel_x_sigma_threshold_ms",
                                             m_d_radar_ego_estimation_vel_x_sigma_threshold_ms) != true)
        return false;
    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_d_radar_ego_estimation_vel_y_sigma_threshold_ms",
                                             m_d_radar_ego_estimation_vel_y_sigma_threshold_ms) != true)
        return false;
    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_d_radar_ego_estimation_vel_z_sigma_threshold_ms",
                                             m_d_radar_ego_estimation_vel_z_sigma_threshold_ms) != true)
        return false;

    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_i_min_input_points_num",
                                             m_i_min_input_points_num) != true)
        return false;

    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_d_radar_calib_x_m", m_d_radar_calib_x_m) != true)
        return false;
    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_d_radar_calib_y_m", m_d_radar_calib_y_m) != true)
        return false;
    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_d_radar_calib_z_m", m_d_radar_calib_z_m) != true)
        return false;
    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_d_radar_calib_roll_deg",
                                             m_d_radar_calib_roll_deg) != true)
        return false;
    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_d_radar_calib_pitch_deg",
                                             m_d_radar_calib_pitch_deg) != true)
        return false;
    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_d_radar_calib_yaw_deg",
                                             m_d_radar_calib_yaw_deg) != true)
        return false;

    if (registration_ini_handler.ParseConfig("PointCloudOdometry", "m_i_m_scan_num", m_i_m_scan_num) != true)
        return false;

    pcl::getTransformation(m_d_radar_calib_x_m, m_d_radar_calib_y_m, m_d_radar_calib_z_m,
                           m_d_radar_calib_roll_deg * M_PI / 180.0, m_d_radar_calib_pitch_deg * M_PI / 180.0,
                           m_d_radar_calib_yaw_deg * M_PI / 180.0, m_tf_radar_calib);

    return true;
}