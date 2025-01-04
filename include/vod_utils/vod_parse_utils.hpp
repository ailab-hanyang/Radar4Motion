#ifndef __VoD_Parse_Utils_HPP__
#define __VoD_Parse_Utils_HPP__

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

// sensor msgs
#include <sensor_msgs/PointCloud2.h>

// custom vod radar point
#include "types.h"

#include <fstream>
#include <filesystem>

class VodParseUtils
{
    public:
        VodParseUtils():
        str_radar_frame_id_("radar") {}
        ~VodParseUtils() = default;

    bool ReadSequenceFile(const std::string& sequence_file_path, std::vector<std::string>& sequence_numbers);
    Eigen::Affine3d ParsePoseJsonFileForSequence(const std::filesystem::path& directory_path, const std::string& sequence_number, const Eigen::Affine3d& cam2radar_tfrom);
    void ParsePoseJsonFiles(const std::vector<std::filesystem::path>& pose_files, const Eigen::Affine3d& cam2radar_tfrom, std::vector<Eigen::Affine3d>& odom2radar_transforms);
    void GetPoseJsonFiles(std::string& pose_directory_path, std::vector<std::filesystem::path>& pose_files);
    void GetPoseJsonFilesWithSeq(const std::string& pose_directory_path, const std::vector<std::string>& sequence_numbers, std::vector<std::filesystem::path>& pose_files);
    void ConvertBin2PointCloud(std::ifstream& bin_file, pcl::PointCloud<RadarPoint>::Ptr& o_point_cloud, int time_idx, sensor_msgs::PointCloud2& o_point_cloud_msg);
    void GetRadarFiles(std::string& radar_directory_path, std::vector<std::filesystem::path>& radar_files);
    
    Eigen::Affine3d jsonToAffine3d(const nlohmann::json& json);
    std::string str_radar_frame_id_;
};
#endif