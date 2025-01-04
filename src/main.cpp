#include <ros/ros.h>
#include "core.cpp" // ImagingRadarOdometryRun

int main(int argc, char **argv)
{
    std::string str_nodename = "radar4motion";
	ros::init(argc, argv, str_nodename);
    ros::NodeHandle nh;
	
    int i_init_loop_frequency = 10;
    
    ros::Rate rosrate_loop_rate(i_init_loop_frequency);
    ImagingRadarOdometryRun<pcl::PointXYZI> imagingRadarOdometryRun;

    bool b_flag_run_vod_dataset = false;
    nh.getParam("/radar4motion/b_flag_run_vod_dataset", b_flag_run_vod_dataset);

    // Run radar odometry with VOD dataset
    if (b_flag_run_vod_dataset)
    {
        ROS_INFO_STREAM("RUNNING RADAR ODOMETRY WITH VOD DATASET");
        imagingRadarOdometryRun.ProcessRadarFiles();
    }
    // Run radar odometry (rostopic)
    else
    {
        ROS_INFO_STREAM("RUNNING RADAR ODOMETRY");
        while(ros::ok())
        {
            imagingRadarOdometryRun.RunOdometry();
            ros::spinOnce();
            rosrate_loop_rate.sleep();
        }
    }

    return 0;
}