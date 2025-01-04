/**
 * @copyright (c) AI LAB - Hanyang Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author soyeongkim@hanyang.ac.kr, jiwonseok@hanyang.ac.kr
 * @file odometry.hpp
 * @brief Radar4tmotion - odometry
 * @version 1.0
 * @date 03-09-2024
 */

#ifndef __CRadar4MotionOdometry_HPP__
#define __CRadar4MotionOdometry_HPP__

// PCL
#include <pcl/common/transforms.h>

#include <math.h>
#include <tf/tf.h>
#include <numeric>
#include <unordered_map>

// INI handler
#include "ini_handler_cpp/c_ini.hpp"

// kiss-icp
#include "kiss-icp/Preprocessing.hpp"
#include "kiss-icp/Registration.hpp"
#include "kiss-icp/Threshold.hpp"
#include "kiss-icp/VoxelHashMap.hpp"

using PointType = pcl::PointXYZI;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;

class CRadar4MotionOdometry {
public:
    CRadar4MotionOdometry()
        : m_d_max_corr_dist(1.0),
          m_i_max_iter_num(20),
          m_leaf_size(0.1),
          m_b_outlier_filter(true),
          m_i_outlier_mean_k(50),
          m_d_outlier_std_threshold(1.0),
          m_submap_leaf_size(0.1),
          m_i_min_input_points_num(5),
          m_d_radar_ego_estimation_vel_x_sigma_threshold_ms(1.0),
          m_d_radar_ego_estimation_vel_y_sigma_threshold_ms(0.1),
          m_d_radar_ego_estimation_vel_z_sigma_threshold_ms(0.001),
          m_result_tf(Eigen::Affine3d::Identity()),
          m_d_prev_timestamp(0.0),
          m_d_kiss_icp_voxel_size_m(1.0),
          m_d_kiss_icp_max_range_m(100.0),
          m_d_kiss_icp_max_points_per_voxel(20),
          m_i_max_num_iterations(500),
          m_d_convergence_criterion(0.0001),
          m_i_num_threads(0),
          m_kiss_icp_voxel_hash_local_map(m_d_kiss_icp_voxel_size_m, m_d_kiss_icp_max_range_m,
                                          m_d_kiss_icp_max_points_per_voxel),
          m_kiss_icp_adaptive_threshold(1.0, 0.1, m_d_kiss_icp_max_range_m),
          m_kiss_icp_registration(m_i_max_num_iterations, m_d_convergence_criterion, m_i_num_threads),
          m_d_corr_dist_weight(3.0),
          m_d_radar_calib_roll_deg(0.),
          m_d_radar_calib_pitch_deg(0.),
          m_d_radar_calib_yaw_deg(0.),
          m_b_use_motion_interpolated_odom(true),
          m_i_submap_scan_num(5),
          m_i_m_scan_num(5) {
        return;
    }

    ~CRadar4MotionOdometry() { return; }

private:
    // ICP parameters
    double m_d_max_corr_dist;
    int m_i_max_iter_num;
    double m_leaf_size;

    bool m_b_use_motion_interpolated_odom;

    bool m_b_outlier_filter;
    int m_i_outlier_mean_k;
    double m_d_outlier_std_threshold;
    double m_submap_leaf_size;

    CINI_H registration_ini_handler;
    std::string m_str_odometry_ini_path;

    double m_d_prev_timestamp;

    double m_d_radar_ego_estimation_vel_x_sigma_threshold_ms;
    double m_d_radar_ego_estimation_vel_y_sigma_threshold_ms;
    double m_d_radar_ego_estimation_vel_z_sigma_threshold_ms;

    // For submap generation (N scans, not in N meters)
    int m_i_submap_scan_num;
    std::vector<Sophus::SE3d> m_submap_poses;
    std::vector<std::vector<Eigen::Vector3d>> m_submap_point_clouds;
    std::vector<std::vector<double>> m_submap_point_rcs;

    // kiss-icp configs
    double m_d_kiss_icp_voxel_size_m;
    double m_d_kiss_icp_max_range_m;
    int m_d_kiss_icp_max_points_per_voxel;
    int m_i_min_input_points_num;

    int m_i_max_num_iterations;
    double m_d_convergence_criterion;
    int m_i_num_threads;

    // KISS-ICP pipeline modules
    // kiss_icp::KISSConfig m_kiss_icp_config;
    kiss_icp::Registration m_kiss_icp_registration;
    kiss_icp::VoxelHashMap m_kiss_icp_voxel_hash_local_map;
    kiss_icp::AdaptiveThreshold m_kiss_icp_adaptive_threshold;

    // kiss-icp results
    std::vector<Sophus::SE3d> m_odom_poses;
    Eigen::Affine3d m_result_tf;
    std::vector<Eigen::Vector3d> m_odom_local_map;

    double m_d_corr_dist_weight;

public:
    // radar calibration configs
    double m_d_radar_calib_x_m;
    double m_d_radar_calib_y_m;
    double m_d_radar_calib_z_m;
    double m_d_radar_calib_roll_deg;
    double m_d_radar_calib_pitch_deg;
    double m_d_radar_calib_yaw_deg;
    int m_i_m_scan_num;
    Eigen::Affine3d m_tf_radar_calib;

public:
    Vector6d m_vec6d_init_ego_motion_estimation;

private:
    void GetXYZRPY(const Sophus::SE3d& i_se3d_tf, Eigen::Affine3d& o_eigen_tf);
    void GetScaledSE3(const Sophus::SE3d& i_se3d_tf, const double scale, Sophus::SE3d& o_se3d);
    void GetMScanAccumulatedPointCloud(const std::vector<Eigen::Vector3d>& i_point_cloud,
                                       const std::vector<double>& i_point_cloud_rcs,
                                       const std::deque<Sophus::SE3d>& init_guesses, const Sophus::SE3d& last_pose,
                                       double time_s, std::vector<Eigen::Vector3d>& M_scan_accumulated_point_cloud,
                                       std::vector<double>& M_scan_accumulated_rcs);
    void ConvertRCSPointPair2PointCloud(const std::vector<std::pair<Eigen::Vector3d, double>>& i_point_rcs_pairs,
                                        std::vector<Eigen::Vector3d>& i_point_cloud,
                                        std::vector<double>& i_point_cloud_rcs);
    Sophus::SE3d GetPredictionModel() const;
    bool HasMoved();
    double GetAdaptiveThreshold();
    bool ParseINIRegistration();

private:
    void PredictEgoMotion(Eigen::Vector3d& i_vec_ego_motion_3dof_vel, Eigen::Vector3d& i_vec_ego_motion_3dof_vel_sigma,
                          double i_curr_point_time, Vector6d& o_pred_motion_6dof);

public:
    bool Init(std::string ini_path);
    bool Init();
    bool RunOdometry(double time_s, std::vector<std::pair<Eigen::Vector3d, double>> i_point_rcs_pairs,
                     Eigen::Vector3d& i_vec_ego_motion_3dof_vel, Eigen::Vector3d& i_vec_ego_motion_3dof_vel_sigma);

    void GetPose(Eigen::Affine3d& result_pose) { result_pose = m_result_tf; };
    void GetSubMap(std::vector<Eigen::Vector3d>& result_map) { result_map = m_odom_local_map; };

    std::vector<Eigen::Vector3d> m_vec_correspondence_src;
    std::vector<Eigen::Vector3d> m_vec_correspondence_tgt;
};

#endif // __CRadar4MotionOdometry_HPP__