/**
 * @copyright (c) AI LAB - Hanyang Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted
 * disclosure only.
 * @author soyeongkim@hanyang.ac.kr
 * @file vod_parse_utils.cpp
 * @brief radar4motion vod dataset parsing source code
 * @version 1.0
 * @date 19-12-2024
 * @bug No known bugs
 * @warning No warnings
 */

#include "vod_parse_utils.hpp"

bool VodParseUtils::ReadSequenceFile(const std::string& sequence_file_path, std::vector<std::string>& sequence_numbers) {
    std::ifstream file(sequence_file_path);
    if (!file.is_open()) {
        ROS_ERROR_STREAM("Unable to open file: " << sequence_file_path);
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            sequence_numbers.push_back(line);
        }
    }

    file.close();
    return true;
}

Eigen::Affine3d VodParseUtils::ParsePoseJsonFileForSequence(const std::filesystem::path& directory_path, const std::string& sequence_number, const Eigen::Affine3d& cam2radar_tfrom)
{
    // Construct the full path to the JSON file using the sequence number
    std::filesystem::path file_path = directory_path / (sequence_number + ".json");

    // Open the file
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return Eigen::Affine3d::Identity();  // Return identity if file cannot be opened
    }

    // Read the JSON file
    nlohmann::json json_data;
    try {
        file >> json_data;
        file.close();
    } catch (const nlohmann::json::parse_error& ex) {
        std::cerr << "Parse error at byte " << ex.byte << ": " << ex.what() << std::endl;
        return Eigen::Affine3d::Identity();  // Return identity if there is a parse error
    }

    // Process the JSON data to extract the transformation
    if (json_data.contains("odomToCamera") && json_data["odomToCamera"].is_array()) {
        Eigen::Affine3d odom2radar = jsonToAffine3d(json_data["odomToCamera"]) * cam2radar_tfrom;
        return odom2radar;  // Return the transformation
    } else {
        std::cerr << "Missing or invalid 'odomToCamera' data in file: " << file_path << std::endl;
        return Eigen::Affine3d::Identity();  // Return identity if data is missing or invalid
    }
}

void VodParseUtils::ParsePoseJsonFiles(const std::vector<std::filesystem::path>& pose_files, const Eigen::Affine3d& cam2radar_tfrom, std::vector<Eigen::Affine3d>& odom2radar_transforms)
{
    odom2radar_transforms.clear();
    odom2radar_transforms.reserve(pose_files.size());

    bool b_first_pose = true;
    Eigen::Affine3d first_odom2radar;

    for (const auto& file_path : pose_files) {
        std::ifstream file(file_path);

        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << file_path << std::endl;
            continue;
        }

        // Read the JSON file
        nlohmann::json json_data;
        try {
            file >> json_data;
            file.close();
        } catch (const nlohmann::json::parse_error& ex) {
            std::cerr << "Parse error at byte "<< ex.byte << ": " << ex.what() << std::endl;
            continue;
        }

        // Checking and logging for each key
        if (json_data.contains("odomToCamera") && json_data["odomToCamera"].is_array()) {
            Eigen::Affine3d odom2radar = jsonToAffine3d(json_data["odomToCamera"]) * cam2radar_tfrom;

            if (b_first_pose)
            {
                first_odom2radar = odom2radar;
                b_first_pose = false;
            }
            odom2radar_transforms.push_back(first_odom2radar.inverse() * odom2radar);
        } else {
            std::cerr << "Missing or invalid 'odomToCamera' data in file: " << file_path << std::endl;
        }
    }
}

Eigen::Affine3d VodParseUtils::jsonToAffine3d(const nlohmann::json& json) {
    if (json.is_null()) {
        std::cerr<<"jsonToAffine3d: json is null"<<std::endl;
        return Eigen::Affine3d::Identity();
    } else {
        Eigen::Matrix4d mat;
        if (json.is_array()) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    mat(i, j) = json[i * 4 + j];
                }
            }
        } else {
            throw std::runtime_error("Expected a JSON array for transformation matrix.");
        }
        return Eigen::Affine3d(mat);
    }
}

void VodParseUtils::GetPoseJsonFiles(std::string& pose_directory_path, std::vector<std::filesystem::path>& pose_files)
{
    pose_files.clear();
    for (const auto& entry : std::filesystem::directory_iterator(pose_directory_path)) {
        if (entry.path().extension() == ".json") {
            pose_files.push_back(entry.path());
        }
    }

    // Sort the files by their sequence number
    std::sort(pose_files.begin(), pose_files.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
        return std::stoi(a.stem()) < std::stoi(b.stem());
    });
}

void VodParseUtils::GetPoseJsonFilesWithSeq(const std::string& pose_directory_path, const std::vector<std::string>& sequence_numbers, std::vector<std::filesystem::path>& pose_files)
{
    pose_files.clear();
    std::set<std::string> sequence_set(sequence_numbers.begin(), sequence_numbers.end());  // Convert list to set for fast lookup

    for (const auto& entry : std::filesystem::directory_iterator(pose_directory_path)) {
        if (entry.path().extension() == ".json") {
            std::string seq_num = entry.path().stem();
            if (sequence_set.find(seq_num) != sequence_set.end()) {
                pose_files.push_back(entry.path());
            }
        }
    }

    // Optionally sort the files by their sequence number
    std::sort(pose_files.begin(), pose_files.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
        return std::stoi(a.stem()) < std::stoi(b.stem());
    });
}


void VodParseUtils::ConvertBin2PointCloud(std::ifstream& bin_file, pcl::PointCloud<RadarPoint>::Ptr& o_point_cloud, int time_idx, sensor_msgs::PointCloud2& o_point_cloud_msg)
{
    o_point_cloud->points.clear();

    VodRadarPointType vod_point;
    while (bin_file.read((char*)&vod_point, sizeof(VodRadarPointType))) {
        RadarPoint radar_point;
        radar_point.x = vod_point.x;
        radar_point.y = vod_point.y;
        radar_point.z = vod_point.z;
        radar_point.vel = vod_point.v_r;
        radar_point.RCS = vod_point.RCS;

        o_point_cloud->push_back(radar_point);
    }
    o_point_cloud->header.stamp = pcl_conversions::toPCL(ros::Time(0.1*time_idx)); // VoD dataset time interval is 0.1s

    // Convert the point cloud to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*o_point_cloud, output);
    output.header.frame_id = str_radar_frame_id_;

    o_point_cloud_msg = output;
}

void VodParseUtils::GetRadarFiles(std::string& radar_directory_path, std::vector<std::filesystem::path>& radar_files)
{
    radar_files.clear();
    
    for (const auto& entry : std::filesystem::directory_iterator(radar_directory_path)) {
        if (entry.path().extension() == ".bin") {
            radar_files.push_back(entry.path());
        }
    }

    // Sort the files by their sequence number
    std::sort(radar_files.begin(), radar_files.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
        return std::stoi(a.stem()) < std::stoi(b.stem());
    });
}