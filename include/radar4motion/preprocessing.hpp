/**
 * @copyright (c) AI LAB - Hanyang Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author soyeongkim@hanyang.ac.kr, jiwonseok@hanyang.ac.kr
 * @file odometry.hpp
 * @brief Radar4tmotion - preprocessing
 * @version 1.0
 * @date 03-09-2024
 */

#ifndef __CRadar4MotionPreprocessing_HPP__
#define __CRadar4MotionPreprocessing_HPP__

#include <fstream>
#include <iostream>
#include <vector>

// json
#include <nlohmann/json.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <fstream>

struct SphericalVoxelKey {
    int azimuthIndex;
    int elevationIndex;
    int radiusIndex;

    bool operator<(const SphericalVoxelKey& other) const {
        return std::tie(azimuthIndex, elevationIndex, radiusIndex) <
               std::tie(other.azimuthIndex, other.elevationIndex, other.radiusIndex);
    }
};

struct SphericalCoordinate {
    double azimuth;
    double elevation;
    double radius;
};

class CRadar4MotionPreprocessing {
public:
    CRadar4MotionPreprocessing()
        : b_local_rcs_filter_(true),
          i_local_rcs_num_(2),
          d_local_rcs_radius_interval_m_(0.5),
          d_local_rcs_azimuth_interval_deg_(1.0),
          d_local_rcs_elevation_interval_deg_(30.0),
          d_local_rcs_minimum_diff_(10.0),
          b_local_rcs_normalization_(true),
          b_crop_points_(false),
          d_x_point_threshold_(-1),
          d_y_point_threshold_(-1),
          d_z_point_threshold_(-1.5) {}

    ~CRadar4MotionPreprocessing() = default;

public:
    SphericalCoordinate CartesianToSpherical(const Eigen::Vector3d& point);
    void LocalRCSFilterPointCloudInSphericalVoxels(
            const std::vector<std::pair<Eigen::Vector3d, double>> i_point_rcs_pairs,
            std::vector<std::pair<Eigen::Vector3d, double>>& o_point_rcs_pairs, double radius_interval_m,
            double azimuth_interval_deg, double elevation_interval_deg, int N);
    void LocalRCSNormalization(const std::vector<std::pair<Eigen::Vector3d, double>>& i_point_rcs_pairs,
                               std::vector<std::pair<Eigen::Vector3d, double>>& o_point_rcs_pairs,
                               double radius_interval_m, double azimuth_interval_deg, double elevation_interval_deg);
    void Preprocessing(const std::vector<std::pair<Eigen::Vector3d, double>>& source_point_rcs_pairs,
                       std::vector<std::pair<Eigen::Vector3d, double>>& output_point_rcs_pairs);
    bool CropPoints(const std::vector<std::pair<Eigen::Vector3d, double>>& i_points,
                    std::vector<std::pair<Eigen::Vector3d, double>>& o_points);

public:
    double d_local_rcs_minimum_diff_;

    bool b_crop_points_;
    double d_x_point_threshold_;
    double d_y_point_threshold_;
    double d_z_point_threshold_;

    bool b_local_rcs_normalization_;

    bool b_local_rcs_filter_;
    int i_local_rcs_num_;
    double d_local_rcs_radius_interval_m_;
    double d_local_rcs_azimuth_interval_deg_;
    double d_local_rcs_elevation_interval_deg_;
};
#endif