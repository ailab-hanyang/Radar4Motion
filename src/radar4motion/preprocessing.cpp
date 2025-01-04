/**
 * @copyright (c) AI LAB - Hanyang Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted
 * disclosure only.
 * @author soyeongkim@hanyang.ac.kr, jiwonseok@hanyang.ac.kr
 * @file preprocessing.cpp
 * @brief radar4motion preprocessing source code
 * @version 1.0
 * @date 19-12-2024
 * @bug No known bugs
 * @warning No warnings
 */

#include "preprocessing.hpp"

void CRadar4MotionPreprocessing::LocalRCSFilterPointCloudInSphericalVoxels(
        const std::vector<std::pair<Eigen::Vector3d, double>> i_point_rcs_pairs,
        std::vector<std::pair<Eigen::Vector3d, double>>& o_point_rcs_pairs, double radius_interval_m,
        double azimuth_interval_deg, double elevation_interval_deg, int N) {
    o_point_rcs_pairs.clear();

    std::map<SphericalVoxelKey, std::vector<std::pair<Eigen::Vector3d, double>>> voxel_map;

    double azimuth_interval_rad = azimuth_interval_deg * M_PI / 180.0;
    double elevation_interval_rad = elevation_interval_deg * M_PI / 180.0;

    // Convert to spherical coorinate & assign point in voxel
    for (size_t i = 0; i < i_point_rcs_pairs.size(); ++i) {
        Eigen::Vector3d i_point_cloud = i_point_rcs_pairs[i].first;
        double i_point_cloud_rcs = i_point_rcs_pairs[i].second;

        SphericalCoordinate spherical = CartesianToSpherical(i_point_cloud);

        // calculate voxel index
        int azimuth_index = static_cast<int>(std::floor(spherical.azimuth / azimuth_interval_rad));
        int elevation_index = static_cast<int>(std::floor(spherical.elevation / elevation_interval_rad));
        int radius_index = static_cast<int>(std::floor(spherical.radius / radius_interval_m));

        // add point & RCS value in voxel
        voxel_map[{azimuth_index, elevation_index, radius_index}].push_back({i_point_cloud, i_point_cloud_rcs});
    }

    // Select top N RCS points
    for (auto& voxel : voxel_map) {
        auto& points_rcs = voxel.second;
        std::sort(points_rcs.begin(), points_rcs.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });

        for (int i = 0; i < std::min(N, static_cast<int>(points_rcs.size())); ++i) {
            o_point_rcs_pairs.push_back(std::make_pair(points_rcs[i].first, points_rcs[i].second));
        }
    }
}

SphericalCoordinate CRadar4MotionPreprocessing::CartesianToSpherical(const Eigen::Vector3d& point) {
    SphericalCoordinate spherical;
    spherical.radius = point.norm();
    spherical.azimuth = std::atan2(point.y(), point.x());
    spherical.elevation = std::atan2(point.z(), std::sqrt(point.x() * point.x() + point.y() * point.y()));
    return spherical;
}

void CRadar4MotionPreprocessing::LocalRCSNormalization(
        const std::vector<std::pair<Eigen::Vector3d, double>>& i_point_rcs_pairs,
        std::vector<std::pair<Eigen::Vector3d, double>>& o_point_rcs_pairs, double radius_interval_m,
        double azimuth_interval_deg, double elevation_interval_deg) {
    o_point_rcs_pairs.clear();
    std::map<SphericalVoxelKey, std::vector<std::pair<Eigen::Vector3d, double>>> voxel_map;

    double azimuth_interval_rad = azimuth_interval_deg * M_PI / 180.0;
    double elevation_interval_rad = elevation_interval_deg * M_PI / 180.0;

    // Convert to spherical coordinates & assign point in voxel
    for (const auto& point_rcs : i_point_rcs_pairs) {
        const Eigen::Vector3d& i_point_cloud = point_rcs.first;
        double i_point_cloud_rcs = point_rcs.second;

        SphericalCoordinate spherical = CartesianToSpherical(i_point_cloud);

        // Calculate voxel index
        int azimuth_index = static_cast<int>(std::floor(spherical.azimuth / azimuth_interval_rad));
        int elevation_index = static_cast<int>(std::floor(spherical.elevation / elevation_interval_rad));
        int radius_index = static_cast<int>(std::floor(spherical.radius / radius_interval_m));

        // Add point & RCS value in voxel
        voxel_map[{azimuth_index, elevation_index, radius_index}].push_back({i_point_cloud, i_point_cloud_rcs});
    }

    // Normalize RCS values within each voxel
    for (auto& voxel : voxel_map) {
        auto& points_rcs = voxel.second;

        // Find the min and max RCS values in the voxel
        double min_rcs = std::numeric_limits<double>::max();
        double max_rcs = std::numeric_limits<double>::lowest();

        for (const auto& point_rcs : points_rcs) {
            double rcs = point_rcs.second;
            if (rcs < min_rcs) min_rcs = rcs;
            if (rcs > max_rcs) max_rcs = rcs;
        }

        // Normalize the RCS values in the voxel
        double maximum_normalized_rcs = 10.0;
        if (max_rcs - min_rcs > d_local_rcs_minimum_diff_) {
            for (const auto& point_rcs : points_rcs) {
                const Eigen::Vector3d& point = point_rcs.first;
                double rcs = point_rcs.second;
                double normalized_rcs = (rcs - min_rcs) * maximum_normalized_rcs / (max_rcs - min_rcs);
                o_point_rcs_pairs.push_back({point, normalized_rcs});
            }
        }
        else {
            for (const auto& point_rcs : points_rcs) {
                const Eigen::Vector3d& point = point_rcs.first;
                double rcs = maximum_normalized_rcs / 2.0; // half maximum value
                o_point_rcs_pairs.push_back({point, rcs});
            }
        }
    }
}

void CRadar4MotionPreprocessing::Preprocessing(
        const std::vector<std::pair<Eigen::Vector3d, double>>& source_point_rcs_pairs,
        std::vector<std::pair<Eigen::Vector3d, double>>& output_point_rcs_pairs) {
    // 2-1. Crop points in ROI
    std::vector<std::pair<Eigen::Vector3d, double>> cropped_source_point_rcs_pairs = source_point_rcs_pairs;
    if (b_crop_points_) {
        CropPoints(source_point_rcs_pairs, cropped_source_point_rcs_pairs);
    }

    // 2-2. Local RCS Filter
    std::vector<std::pair<Eigen::Vector3d, double>> local_rcs_filtered_point_rcs_pairs = cropped_source_point_rcs_pairs;
    if (b_local_rcs_filter_) {
        LocalRCSFilterPointCloudInSphericalVoxels(cropped_source_point_rcs_pairs, local_rcs_filtered_point_rcs_pairs,
                                                  d_local_rcs_radius_interval_m_, d_local_rcs_azimuth_interval_deg_,
                                                  d_local_rcs_elevation_interval_deg_, i_local_rcs_num_);
    }

    // 2-3. Normalize RCS
    std::vector<std::pair<Eigen::Vector3d, double>> normalized_point_rcs_pairs = local_rcs_filtered_point_rcs_pairs;

    if (b_local_rcs_normalization_) {
        LocalRCSNormalization(local_rcs_filtered_point_rcs_pairs, normalized_point_rcs_pairs, 10.0, 45.0, 30.0);
    }

    output_point_rcs_pairs = normalized_point_rcs_pairs;
}

bool CRadar4MotionPreprocessing::CropPoints(const std::vector<std::pair<Eigen::Vector3d, double>>& i_points,
                                            std::vector<std::pair<Eigen::Vector3d, double>>& o_points) {
    o_points.clear();

    if (i_points.size() == 0) {
        std::cout << "[CropPoints] Input io_points are empty!" << std::endl;
        return false;
    }

    for (auto& i_point : i_points) {
        if ((fabs(i_point.first(0)) < 5.0 && fabs(i_point.first(1)) < 5.0) == false) {
            if (i_point.first(2) > d_z_point_threshold_ && i_point.first(2) < 5.0 &&
                fabs(i_point.first(0)) < d_x_point_threshold_ && fabs(i_point.first(1)) < d_y_point_threshold_) {
                o_points.push_back(i_point);
            }
        }
    }

    return true;
}