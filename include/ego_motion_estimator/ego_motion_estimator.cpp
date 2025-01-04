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

#define PCL_NO_PRECOMPILE

#include <random>
#include <algorithm>

#include <angles/angles.h>
#include <ego_motion_estimator/ego_motion_estimator.h>


using namespace reve;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(RadarPointCloudType,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, power, power)
                                  (float, vel,   vel)
                                  (float, RCS,   RCS)
                                  )
// clang-format on

static RadarPointCloudType toRadarPointCloudType(const Vector10& item, const RadarEgoVelocityEstimatorIndices& idx)
{
  RadarPointCloudType point;
  point.x             = item[idx.x_r];
  point.y             = item[idx.y_r];
  point.z             = item[idx.z_r];
  point.vel           = -item[idx.v_d];
  point.power        = item[idx.power];
  point.RCS        = item[idx.rcs];
  return point;
}

bool RadarEgoVelocityEstimator::Init(std::string ini_path)
{
    if (m_ini_handler.Init(ini_path.c_str()) == false)
        return false;
    if (ParseINI() == false)
        return false;
    return true;
}

bool RadarEgoVelocityEstimator::ParseINI()
{
    if(m_ini_handler.IsFileUpdated() == false)
        return true;

    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "min_dist", config_.min_dist)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "max_dist", config_.max_dist)               != true)	return false;

    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "elevation_thresh_deg", config_.elevation_thresh_deg)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "azimuth_thresh_deg", config_.azimuth_thresh_deg)               != true)	return false;

    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "filter_min_z", config_.filter_min_z)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "filter_max_z", config_.filter_max_z)               != true)	return false;

    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "doppler_velocity_correction_factor", config_.doppler_velocity_correction_factor)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "thresh_zero_velocity", config_.thresh_zero_velocity)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "allowed_outlier_percentage", config_.allowed_outlier_percentage)               != true)	return false;

    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "sigma_zero_velocity_x", config_.sigma_zero_velocity_x)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "sigma_zero_velocity_y", config_.sigma_zero_velocity_y)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "sigma_zero_velocity_z", config_.sigma_zero_velocity_z)               != true)	return false;

    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "sigma_offset_radar_x", config_.sigma_offset_radar_x)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "sigma_offset_radar_y", config_.sigma_offset_radar_y)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "sigma_offset_radar_z", config_.sigma_offset_radar_z)               != true)	return false;
    
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "max_sigma_x", config_.max_sigma_x)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "max_sigma_y", config_.max_sigma_y)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "max_sigma_z", config_.max_sigma_z)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "use_cholesky_instead_of_bdcsvd", config_.use_cholesky_instead_of_bdcsvd)               != true)	return false;

    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "use_ransac", config_.use_ransac)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "outlier_prob", config_.outlier_prob)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "success_prob", config_.success_prob)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "N_ransac_points", config_.N_ransac_points)               != true)	return false;
    if(m_ini_handler.ParseConfig("RadarEgoMotionEstimation", "inlier_thresh", config_.inlier_thresh)               != true)	return false;

    setRansacIter();
    
    return true;
}

bool RadarEgoVelocityEstimator::estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                                         Vector3& v_r,
                                         Vector3& sigma_v_r)
{
  Matrix3 P_v_r;
  sensor_msgs::PointCloud2 inlier_radar_msg;
  const auto success = estimate(radar_scan_msg, v_r, P_v_r, inlier_radar_msg);
  sigma_v_r          = Vector3(P_v_r(0, 0), P_v_r(1, 1), P_v_r(2, 2)).array().sqrt();
  return success;
}

bool RadarEgoVelocityEstimator::estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                                         Vector3& v_r,
                                         Vector3& sigma_v_r,
                                         sensor_msgs::PointCloud2& inlier_radar_msg)
{
  Matrix3 P_v_r;
  pcl::PointCloud<RadarPointCloudType> radar_scan_inlier;
  bool success = estimate(radar_scan_msg, v_r, P_v_r, radar_scan_inlier);

  pclToPcl2msg(radar_scan_inlier, inlier_radar_msg);
  inlier_radar_msg.header = radar_scan_msg.header;
  sigma_v_r               = Vector3(P_v_r(0, 0), P_v_r(1, 1), P_v_r(2, 2)).array().sqrt();

  return success;
}

bool RadarEgoVelocityEstimator::estimate(const sensor_msgs::PointCloud2& radar_scan_msg, Vector3& v_r, Matrix3& P_v_r)
{
  sensor_msgs::PointCloud2 inlier_radar_msg;
  return estimate(radar_scan_msg, v_r, P_v_r, inlier_radar_msg);
}

bool RadarEgoVelocityEstimator::estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                                         Vector3& v_r,
                                         Matrix3& P_v_r,
                                         sensor_msgs::PointCloud2& inlier_radar_msg)
{
  pcl::PointCloud<RadarPointCloudType> radar_scan_inlier;
  bool success = estimate(radar_scan_msg, v_r, P_v_r, radar_scan_inlier);

  pclToPcl2msg(radar_scan_inlier, inlier_radar_msg);
  inlier_radar_msg.header = radar_scan_msg.header;

  return success;
}

bool RadarEgoVelocityEstimator::estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                                         Vector3& v_r,
                                         Matrix3& P_v_r,
                                         pcl::PointCloud<RadarPointCloudType>& radar_scan_inlier,
                                         const Matrix3& C_stab_r)
{
  auto radar_scan(new pcl::PointCloud<RadarPointCloudType>);

  bool success = false;

  if (pcl2msgToPcl(radar_scan_msg, *radar_scan))
    return false;
  // Radar Point Cloud validation with thresholding parameter (distance, power, angle, height)
  std::vector<Vector10> valid_targets;
  for (uint i = 0; i < radar_scan->size(); ++i)
  {
    const auto target = radar_scan->at(i);
    const Real r      = Vector3(target.x, target.y, target.z).norm();
    if (r < config_.min_dist || r > config_.max_dist)
      continue;
    Real azimuth   = std::atan2(target.y, target.x);
    Real elevation = std::atan2(target.z, std::sqrt(target.x * target.x + target.y * target.y));

    if (std::fabs(azimuth) > angles::from_degrees(config_.azimuth_thresh_deg) ||
        std::fabs(elevation) > angles::from_degrees(config_.elevation_thresh_deg))
      continue;
    const Vector3 p_stab = C_stab_r * Vector3(target.x, target.y, target.z);

    if (p_stab.z() < config_.filter_min_z || p_stab.z() > config_.filter_max_z)
      continue;
    Vector10 v;
    v << target.x, target.y, target.z, target.power, target.RCS, -target.vel * config_.doppler_velocity_correction_factor, 
        target.x / r, target.y / r, target.z / r, r;
    valid_targets.emplace_back(v);
  }

  if (valid_targets.size() < 2)
    return false;

  // check for zero velocity
  std::vector<Real> v_dopplers;
  for (const auto& v : valid_targets)
    v_dopplers.emplace_back(std::fabs(v[idx_.v_d]));
  const size_t n = v_dopplers.size() * (1.0 - config_.allowed_outlier_percentage);
  std::nth_element(v_dopplers.begin(), v_dopplers.begin() + n, v_dopplers.end());
  const auto median = v_dopplers[n]; // jiwon: n is not median n is inlier largest velocity

  if (median < config_.thresh_zero_velocity) // Zero velocity case
  {
    ROS_INFO_STREAM_THROTTLE(0.5, kPrefix << "Zero velocity detected!");
    // Zero velocity output generation
    v_r = Vector3(0, 0, 0);
    P_v_r.setIdentity();
    P_v_r.diagonal() =
        Vector3(config_.sigma_zero_velocity_x, config_.sigma_zero_velocity_y, config_.sigma_zero_velocity_z)
            .array()
            .square();

    // Zero velocity inlier point extraction`
    for (const auto& item : valid_targets)
      if (std::fabs(item[idx_.v_d]) < config_.thresh_zero_velocity)
        radar_scan_inlier.push_back(toRadarPointCloudType(item, idx_));

    success = true;
  }
  else
  {
    // LSQ velocity estimation
    Matrix radar_data(valid_targets.size(), 4);  // rx, ry, rz, v
    uint idx = 0;
    for (const auto& v : valid_targets){
      radar_data.row(idx++) = Vector4(v[idx_.r_x], v[idx_.r_y], v[idx_.r_z], v[idx_.v_d]);
    }

    if (config_.use_ransac) // jiwon: is there any other option without ransac, why use this..
    {
      std::vector<uint> inlier_idx_best;
      if(solve3DLsqRansac(radar_data, v_r, P_v_r, inlier_idx_best) == false)
        return false;

      for (const auto& idx : inlier_idx_best)
        radar_scan_inlier.push_back(toRadarPointCloudType(valid_targets.at(idx), idx_));
    }
  }

  return success;
}

bool RadarEgoVelocityEstimator::estimateWithInitialVelocity(const sensor_msgs::PointCloud2 &radar_scan_msg, 
                                                                  Vector3& init_v_r,
                                                                  Vector3& v_r,
                                                                  Vector3& sigma_v_r,
                                                                  sensor_msgs::PointCloud2& radar_scan_inlier_msg,
                                                                  sensor_msgs::PointCloud2& radar_scan_outlier_msg)
{
  ParseINI();
  pcl::PointCloud<RadarPointCloudType>::Ptr radar_scan(new pcl::PointCloud<RadarPointCloudType>);
  // ============================================================================================
  // Data read
  if (pcl2msgToPcl(radar_scan_msg, *radar_scan)==false)
    return false;
  
  // ============================================================================================
  // Radar Point Cloud validation with thresholding parameter (distance, power, angle, height)
  std::vector<Vector10> valid_targets;
  for (uint i = 0; i < radar_scan->size(); ++i)
  {
    const auto target = radar_scan->at(i);
    const Real r      = Vector3(target.x, target.y, target.z).norm();

    Real azimuth   = std::atan2(target.y, target.x);
    Real elevation = std::atan2(target.z, std::sqrt(target.x * target.x + target.y * target.y));

    const Vector3 p_stab = Vector3(target.x, target.y, target.z);
    
    Vector10 v;
    v << target.x, target.y, target.z, target.power, target.RCS, -target.vel * config_.doppler_velocity_correction_factor, 
        target.x / r, target.y / r, target.z / r, r;
    valid_targets.emplace_back(v);
  }

  if (valid_targets.size() < 10)
    return false;

  // ============================================================================================
  // LSQ velocity estimation
  Matrix radar_data(valid_targets.size(), 4);  // rx, ry, rz, v
  uint idx = 0;
  for (const auto& v : valid_targets){
    radar_data.row(idx++) = Vector4(v[idx_.r_x], v[idx_.r_y], v[idx_.r_z], v[idx_.v_d]);
  }

 std::vector<uint> inlier_idx_best;
  Matrix3 P_v_r;
  {
    Matrix H_all(radar_data.rows(), 3); // x, y, z
    H_all.col(0)       = radar_data.col(0);
    H_all.col(1)       = radar_data.col(1);
    H_all.col(2)       = radar_data.col(2);
    const Vector y_all = radar_data.col(3); // velocity vector

    // Initial inlier estimation with input velocity
    Matrix radar_data_iter; // x, y, z, v
    {
      const Vector err = (y_all - H_all * init_v_r).array().abs();
      std::vector<uint> inlier_idx;
      for (uint j = 0; j < err.rows(); ++j){
        if (err(j) < config_.inlier_thresh)
          inlier_idx.emplace_back(j);
      }
      Matrix initial_radar_data(inlier_idx.size(), 4); // x, y, z, v
      for (uint i = 0; i < inlier_idx.size(); ++i)
        initial_radar_data.row(i) = radar_data.row(inlier_idx.at(i));
      radar_data_iter.swap(initial_radar_data);
    }

    if(radar_data_iter.rows() < config_.N_ransac_points){
      solve3DLsqRansac(radar_data, v_r, P_v_r, inlier_idx_best);
    }
    else{
      // Iterative estimation
      std::vector<uint> inlier_idx;
      for (uint k = 0; k < config_.N_iterative_estimation_iter; ++k)
      { 
        // Solve SAC iteratively
        if (solve3DLsq(radar_data_iter, v_r, P_v_r, true))
        {
          const Vector err = (y_all - H_all * v_r).array().abs();
          inlier_idx.clear();
          for (uint j = 0; j < err.rows(); ++j){
            if (err(j) < config_.inlier_thresh)
              inlier_idx.emplace_back(j);
          }
          Matrix initerative_radar_data(inlier_idx.size(), 4); // x, y, z, v
          for (uint i = 0; i < inlier_idx.size(); ++i)
            initerative_radar_data.row(i) = radar_data.row(inlier_idx.at(i));
          radar_data_iter.swap(initerative_radar_data);
          if (inlier_idx.size() > inlier_idx_best.size())
            inlier_idx_best = inlier_idx;
        }
      }
    }
  }

  // ============================================================================================
  // Extract inlier(static) and outlier(dynamic)
  std::set<uint> inlier_idx_key;
  pcl::PointCloud<RadarPointCloudType> radar_scan_inlier;
  pcl::PointCloud<RadarPointCloudType> radar_scan_outlier;

  for (const auto& idx : inlier_idx_best)
    inlier_idx_key.insert(idx);

  for(int idx = 0; idx < valid_targets.size(); idx++)
  {
    if(inlier_idx_key.find(idx) == inlier_idx_key.end()) // outlier
      radar_scan_outlier.push_back(toRadarPointCloudType(valid_targets.at(idx), idx_));
    else
      radar_scan_inlier.push_back(toRadarPointCloudType(valid_targets.at(idx), idx_));
  }
  
  pclToPcl2msg(radar_scan_inlier, radar_scan_inlier_msg);
  radar_scan_inlier_msg.header = radar_scan_msg.header;
  pclToPcl2msg(radar_scan_outlier, radar_scan_outlier_msg);
  radar_scan_outlier_msg.header = radar_scan_msg.header;

  sigma_v_r = Vector3(P_v_r(0, 0), P_v_r(1, 1), P_v_r(2, 2)).array().sqrt();

  return true;
}
bool RadarEgoVelocityEstimator::solve3DLsqRansac(const Matrix &radar_data,
                                                 Vector3 &v_r,
                                                 Matrix3 &P_v_r,
                                                 std::vector<uint> &inlier_idx_best)
{
  Matrix H_all(radar_data.rows(), 3); // x, y, z
  H_all.col(0)       = radar_data.col(0);
  H_all.col(1)       = radar_data.col(1);
  H_all.col(2)       = radar_data.col(2);
  const Vector y_all = radar_data.col(3); // velocity vector

  // Index vector generation
  std::vector<uint> idx(radar_data.rows());
  for (uint k = 0; k < radar_data.rows(); ++k)
    idx[k] = k;


  if (radar_data.rows() < config_.N_ransac_points)
    return false;

  // RANSAC 
  std::random_device rd;
  std::mt19937 g(rd());
  for (uint k = 0; k < ransac_iter_; ++k)
  {
    // Random generation
    std::shuffle(idx.begin(), idx.end(), g);
    
    // Random point generation
    Matrix radar_data_iter(config_.N_ransac_points, 4); // x, y, z, v
    for (uint i = 0; i < config_.N_ransac_points; ++i)
      radar_data_iter.row(i) = radar_data.row(idx.at(i));

    // Solve SAC
    if (solve3DLsq(radar_data_iter, v_r, P_v_r, true))
    {
      const Vector err = (y_all - H_all * v_r).array().abs();
      std::vector<uint> inlier_idx;
      for (uint j = 0; j < err.rows(); ++j){
        if (err(j) < config_.inlier_thresh)
          inlier_idx.emplace_back(j);
      }
      if (inlier_idx.size() > inlier_idx_best.size())
        inlier_idx_best = inlier_idx;
    }
  }

  if (!inlier_idx_best.empty())
  {
    Matrix radar_data_inlier(inlier_idx_best.size(), 4);
    for (uint i = 0; i < inlier_idx_best.size(); ++i) radar_data_inlier.row(i) = radar_data.row(inlier_idx_best.at(i));

    return solve3DLsq(radar_data_inlier, v_r, P_v_r, true);
  }

  return false;
}

bool RadarEgoVelocityEstimator::solve3DLsq(const Matrix& radar_data, Vector3& v_r, Matrix3& P_v_r, bool estimate_sigma)
{
  Matrix H(radar_data.rows(), 3);
  H.col(0)         = radar_data.col(0);
  H.col(1)         = radar_data.col(1);
  H.col(2)         = radar_data.col(2);
  const Matrix HTH = H.transpose() * H; // covariance of radar random points

  const Vector y = radar_data.col(3);

  Eigen::JacobiSVD<Matrix> svd(HTH);
  Real cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

  // if (std::fabs(cond) < 1.0e3)
  if (1)
  {
    if (config_.use_cholesky_instead_of_bdcsvd)
      v_r = (HTH).ldlt().solve(H.transpose() * y);
    else
      v_r = H.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);

    if (estimate_sigma)
    {
      const Vector e    = H * v_r - y;
      P_v_r             = (e.transpose() * e).x() * (HTH).inverse() / (H.rows() - 3);
      Vector3 sigma_v_r = Vector3(P_v_r(0, 0), P_v_r(1, 1), P_v_r(2, 2));

      const Vector3 offset =
          Vector3(config_.sigma_offset_radar_x, config_.sigma_offset_radar_y, config_.sigma_offset_radar_z)
              .array()
              .square();
      P_v_r += offset.asDiagonal();

      // check diagonal for valid estimation result
      if (sigma_v_r.x() >= 0.0 && sigma_v_r.y() >= 0.0 && sigma_v_r.z() >= 0.)
      {
        sigma_v_r = sigma_v_r.array().sqrt();
        if (sigma_v_r.x() < config_.max_sigma_x && sigma_v_r.y() < config_.max_sigma_y &&
            sigma_v_r.z() < config_.max_sigma_z)
          return true;
      }
    }
    else
    {
      return true;
    }
  }

  return false;
}