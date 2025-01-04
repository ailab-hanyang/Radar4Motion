#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

struct SVertex{
    double time_s;
    Eigen::Affine3d delta_pose_tf;
    double motion[6]; // x_m/s, y_m/s, z_m/s, roll_rad/s, pitch_rad/s, yaw_rad/s;
};

struct RadarPoint
{
    PCL_ADD_POINT4D;      // position in [m]
    float power;         // CFAR cell to side noise ratio in [dB]
    float vel;  // Doppler velocity in [m/s]
    float range;          // range in [m]
    float RCS;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(RadarPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, power, power)
                                  (float, vel,   vel)
                                  (float, range,   range)
                                  (float, RCS,   RCS)
                                  )

struct VodRadarPointType {
    float x, y, z;       // 위치
    float RCS;           // 레이다 반사율
    float v_r;           // 속도
    float v_r_compensated; // 보상된 속도
    float time;          // 측정 시간
};

struct EuclideanCoordinate {
    double x;
    double y;
    double z;
};