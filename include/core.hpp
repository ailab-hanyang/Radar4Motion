/**
 * @copyright (c) AI LAB - Hanyang Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author soyeongkim@hanyang.ac.kr, jiwonseok@hanyang.ac.kr
 * @file core.hpp
 * @brief core function w/ ROS dependencies
 * @version 1.0
 * @date 03-09-2024
 */

#ifndef __Odometry_HPP__
#define __Odometry_HPP__

// ROS header
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"

// Utility header
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> // pcl_conversions::toPCL

#include <std_msgs/Float32MultiArray.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <filesystem>
#include <fstream>

// Sensor_msgs
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// PCL
#include <pcl/filters/voxel_grid.h>

// Rviz
#include <visualization_msgs/Marker.h>

// json
#include <nlohmann/json.hpp>

// Visualization
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// ego motion estimation
#include "ego_motion_estimator/ego_motion_estimator.h"
#include "ego_motion_estimator/radar_point_cloud.h"

// Radar4Motion
#include "odometry.hpp"
#include "preprocessing.hpp"

// INI handler
#include "ini_handler_cpp/c_ini.hpp"

// Vod parse utils
#include "vod_utils/vod_parse_utils.hpp"

template <typename pointT>
class ImagingRadarOdometryRun {
public:
    ImagingRadarOdometryRun();
    ~ImagingRadarOdometryRun();
    bool Initialization();

private:
    bool b_radar_scan_exist_ = false;

    // Odometry results
    Eigen::Affine3d affine3d_result_tf_;

    // Classes
public:
    // Preprocessing
    CRadar4MotionPreprocessing c_radar4motion_preprocessing_;

    // Odometry
    CRadar4MotionOdometry c_radar4motion_odom_;

    // Utils
    VodParseUtils c_vod_parser_;

    // Ego motion estimation (REVE)
    reve::RadarEgoVelocityEstimator c_radar_ego_motion_vel_estimator_;

    // ROS variable
private:
    ros::Subscriber rossub_radar_pointcloud2_;

    ros::Publisher rospub_radar_odom_pose_;
    ros::Publisher rospub_radar_only_odom_pose_;
    ros::Publisher rospub_radar_odom_pose_stamped_;
    ros::Publisher rospub_odom_accumulated_map_;
    ros::Publisher rospub_odom_accumulated_que_partial_map_;
    ros::Publisher rospub_static_radar_points_;
    ros::Publisher rospub_rcs_filtered_points_;
    ros::Publisher rospub_vod_radar_points_;
    ros::Publisher rospub_gt_odom_pose_;

    std::string save_odom_csv_path_;

    sensor_msgs::PointCloud2 i_point_cloud_;
    geometry_msgs::PoseArray pos_arr_odom_;
    geometry_msgs::PoseArray vod_gt_arr_;

    // Ini params
private:
    CINI_H odometry_ini_handler_;

    // launch param
private:
    std::string s_input_points_;

    std::string str_origin_;
    std::string str_base_link_;

    std::string str_rcs_field_name_;
    std::string str_power_field_name_;

    bool b_accum_whole_point_clouds_;
    int i_min_static_points_num_;

    int i_vod_frame_rate_;

    // map management
private:
    // For map management
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> deque_transformed_points_; // vector of <time[sec], pose>
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_radar_odom_submap_;

    // ego motion estimation
    Eigen::Vector3d vec3d_radar_ego_motion_init_vel_;
    Eigen::Vector3d vec3d_radar_ego_motion_init_vel_sigma_;

public:
    void RunOdometry();
    bool ParseINI();

private:
    // Callback functions
    void CallbackRadarPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    void Radar4MotionOdometry(double time_sec, std::vector<std::pair<Eigen::Vector3d, double>>& i_point_rcs_pairs,
                              Eigen::Vector3d& estimated_ego_motion, Eigen::Vector3d& estimated_ego_motion_sigma);
    void RadarEgoMotionEstimation(const sensor_msgs::PointCloud2& input_radar_msg,
                                  sensor_msgs::PointCloud2& inlier_radar_msg, Eigen::Vector3d& estimated_ego_motion,
                                  Eigen::Vector3d& estimated_ego_motion_sigma);

    // Radar point cloud processing
    void GlobalMapAccumulation(const boost::shared_ptr<pcl::PointCloud<pointT>> input_point_cloud,
                               Eigen::Affine3d& radar_pose);

    // Preprocessing
    bool Downsampling(boost::shared_ptr<pcl::PointCloud<pointT>> input_point_cloud,
                      boost::shared_ptr<pcl::PointCloud<pointT>>& output_point_cloud, double leaf_size);

    // Visualization
    void BroadcastTFAndVisualizeOdomPose(const Eigen::Affine3d& result_tf_in_ego_pose, const std::string& str_origin,
                                         const std::string& str_base_link, const std_msgs::Header& point_cloud_header);
    void VisualizeAndPubOdomPose(double odom_x, double odom_y, double odom_z, tf::Quaternion odom_rpy);
    void PublishSubmapPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& submap_point_cloud);

    // Save results in .csv format
    void SaveOdomResult(Eigen::Matrix4d input, ros::Time curr_time);

    // For VoD dataset
public:
    void ProcessRadarFiles();

private:
    void ProcessRadarFile(const std::string& vod_seq_folder_path, const std::string& vod_eval_folder_path,
                          const std::vector<std::string>& sequence_numbers, const int sequence_num);
    void SavePosesToFile(const std::vector<Eigen::Affine3d>& poses, const std::string& filename);

    // Pub GT
    void PubVodGtPose(const Eigen::Affine3d& vod_gt_pose, const std_msgs::Header& header);

    // Calibration params
    Eigen::Matrix4d mat4d_vod_cam2radar_mat_;

private:
    // Conversion functions
    void EigenPointRCSToPointCloud(const std::vector<std::pair<Eigen::Vector3d, double>>& vec_point_rcs_pairs,
                                   boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> output_point_cloud);
    void EigenToPointCloud(const std::vector<Eigen::Vector3d>& vec_point_cloud,
                           boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> output_point_cloud);
    std::vector<std::pair<Eigen::Vector3d, double>> PointCloud2PointsRCSPairs(const sensor_msgs::PointCloud2& msg);
};
#endif