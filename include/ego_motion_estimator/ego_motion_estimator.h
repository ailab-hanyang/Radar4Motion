// This file is part of REVE - Radar Ego Velocity Estimator
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>

#include <ego_motion_estimator/data_types.h>
#include <ego_motion_estimator/radar_point_cloud.h>
#include <ego_motion_estimator/ros_helper.h>

// INI handler
#include "ini_handler_cpp/c_ini.hpp"

namespace reve
{
struct RadarEgoVelocityEstimatorIndices
{
  uint x_r       = 0;
  uint y_r       = 1;
  uint z_r       = 2;
  uint power     = 3;
  uint rcs       = 4;
  uint v_d       = 5;
  uint r_x       = 6;
  uint r_y       = 7;
  uint r_z       = 8;
};

struct RadarEgoVelocityEstimatorConfig
{
 float min_dist = 1;
 float max_dist = 80; // middle range radar spec
 float min_db = 7;
 float elevation_thresh_deg = 30.0;
 float azimuth_thresh_deg = 60.0;
 float filter_min_z = - 0.5;
 float filter_max_z = 10.0;
 float doppler_velocity_correction_factor = 1;
 
 float thresh_zero_velocity = 0.2; //Below this is recognized as inlier (m/s)
 float allowed_outlier_percentage = 0.10;
 // afi910 velocity accuracy 0.1kph = 0.0277778mps
 float sigma_zero_velocity_x = 0.0277778;
 float sigma_zero_velocity_y = 0.0277778;
 float sigma_zero_velocity_z = 0.0277778;
 
 float sigma_offset_radar_x = 0;
 float sigma_offset_radar_y = 0;
 float sigma_offset_radar_z = 0;

 float max_sigma_x = 0.2;
 float max_sigma_y = 0.2;
 float max_sigma_z = 0.2;
 float max_r_cond = 1.0e3;
 bool use_cholesky_instead_of_bdcsvd = true;

 bool use_ransac = true;
 float outlier_prob = 0.3; // Outlier Probability, to calculate ransac_iter_
 float success_prob = 0.995;
 int N_ransac_points = 5;
 float inlier_thresh = 1.0; // err(j) threshold, 0.1 too small, 1.0 too large

 bool use_odr = false;
 float sigma_v_d = 0.125;
 float min_speed_odr = 4.0;
 float model_noise_offset_deg = 2.0;
 float model_noise_scale_deg = 10.0;

 int N_iterative_estimation_iter = 3;
};

class RadarEgoVelocityEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief RadarEgoVelocityEstimator constructor
   */
  RadarEgoVelocityEstimator() {}

  /**
   * @brief Reconfigure callback
   * @param config  has to contain RadarEgoVelocityEstimatorConfig
   * @return
   */
  template <class ConfigContainingRadarEgoVelocityEstimatorConfig>
  bool configure(ConfigContainingRadarEgoVelocityEstimatorConfig& config);
  bool Init(std::string ini_path);
  bool ParseINI();

  /**
   * @brief Estimates the radar ego velocity based on a single radar scan
   * @param[in] radar_scan_msg       radar scan
   * @param[out] v_r                 estimated radar ego velocity
   * @param[out] sigma_v_r           estimated sigmas of ego velocity
   * @param[out] inlier_radar_scan   inlier point cloud
   * @returns true if estimation successful
   */
  bool estimate(const sensor_msgs::PointCloud2& radar_scan_msg, Vector3& v_r, Matrix3& P_v_r);
  bool estimate(const sensor_msgs::PointCloud2& radar_scan_msg, Vector3& v_r, Vector3& sigma_v_r);
  bool estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                Vector3& v_r,
                Matrix3& P_v_r,
                sensor_msgs::PointCloud2& inlier_radar_msg);
  bool estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                Vector3& v_r,
                Vector3& sigma_v_r,
                sensor_msgs::PointCloud2& inlier_radar_msg);
  bool estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                Vector3& v_r,
                Matrix3& P_v_r,
                pcl::PointCloud<RadarPointCloudType>& inlier_radar,
                const Matrix3& C_stab_r = Matrix3::Identity());
  
  /**
* @brief Estimates the radar ego veocity based on a single radar scan with inital velocity estimation
   * @param[in] radar_scan_msg        radar scan
   * @param[in] init_v_r              initial estimated radar ego velocity
   * @param[in] init_sigma_v_r        initial estimated sigmas of ego velocity
   * @param[out] v_r                  estimated radar ego velocity
   * @param[out] sigma_v_r            estimated sigmas of ego velocity
   * @param[out] radar_scan_inlier    static radar point cloud
   * @param[out] radar_scan_outlier   dynamic radar point cloud
   * @return true 
   * @return false 
   */
  bool estimateWithInitialVelocity(const sensor_msgs::PointCloud2& radar_scan_msg,
                                  Vector3& init_v_r,
                                  Vector3& v_r,
                                  Vector3& sigma_v_r,
                                  sensor_msgs::PointCloud2& radar_scan_inlier_msg,
                                  sensor_msgs::PointCloud2& radar_scan_outlier_msg);

private:
  /**
   * @brief Implementation of the ransac based estimation
   * @param[in] radar_data          matrix of parsed radar scan --> see
   * RadarEgoVelocityEstimatorIndices
   * @param[out] v_r                estimated radar ego velocity
   * @param[out] sigma_v_r          estimated sigmas of ego velocity
   * @param[out] inlier_idx_best    idices of inlier
   * @returns true if estimation successful
   */
  bool solve3DLsqRansac(const Matrix& radar_data, Vector3& v_r, Matrix3& P_v_r, std::vector<uint>& inlier_idx_best);

  /**
   * @brief Estimates the radar ego velocity using all mesurements provided in
   * radar_data
   * @param[in] radar_data          matrix of parsed radar scan --> see
   * RadarEgoVelocityEstimatorIndices
   * @param[out] v_r                estimated radar ego velocity
   * @param[out] sigma_v_r          estimated sigmas of ego velocity
   * @param estimate_sigma          if true sigma will be estimated as well
   * @returns true if estimation successful
   */
  bool solve3DLsq(const Matrix& radar_data, Vector3& v_r, Matrix3& P_v_r, bool estimate_sigma = true);

  bool solve3DOdr(const Matrix& radar_data, Vector3& v_r, Matrix3& P_v_r);
  /**
   * @brief Helper function which estiamtes the number of RANSAC iterations
   */
  void setRansacIter()
  {
    ransac_iter_ = uint((std::log(1.0 - config_.success_prob)) /
                        std::log(1.0 - std::pow(1.0 - config_.outlier_prob, config_.N_ransac_points)));
  }

  const std::string kPrefix = "[RadarEgoVelocityEstimator]: ";
  const RadarEgoVelocityEstimatorIndices idx_;

  RadarEgoVelocityEstimatorConfig config_;
  uint ransac_iter_ = 0;

  // Configuration
  CINI_H          m_ini_handler;
};
}  // namespace reve
