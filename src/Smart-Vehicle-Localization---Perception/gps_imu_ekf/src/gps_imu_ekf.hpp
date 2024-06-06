#ifndef GPS_IMU_EKF_H
#define GPS_IMU_EKF_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <lanelet2_projection/UTM.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cmath>

using namespace Eigen;

const int N = 3, M = 2; //N =  state (x, y, yaw), M = Measure (x, y)

MatrixXd Q(N,N), I(N,N), F(N,N), P_post(N,N), P_prior(N,N), P_predict(N,N), K(N,M), R(M,M), H(M,N);
VectorXd f(N), x_post(N), x_predict(N), x_prior(N), z(M), h(M); 

struct utm
{
    double x, y, yaw, prev_x, prev_y, prev_yaw, velocity;
};

class ExtendedKalmanFilter{
    protected:
        ros::NodeHandle nh;

        ros::Publisher m_pose_pub;
        ros::Publisher m_visual_pub;
        ros::Publisher gps_path_pub;
        ros::Publisher dr_path_pub;
        ros::Publisher ekf_path_pub;

        ros::Subscriber m_gps_sub;
        ros::Subscriber m_imu_sub;

    public:
        ExtendedKalmanFilter();
        ~ExtendedKalmanFilter();

        void init();

        void state_init();

        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

        void EKF();

        void DeadReckoning();

        void Visualization(geometry_msgs::PoseStamped gps_pose, geometry_msgs::PoseStamped dr_pose, geometry_msgs::PoseStamped ekf_pose);

        void publishPose();

        double getVehicleSpeed(utm utm);

        bool gps_input, state_init_check;

        double yaw_rate, yaw, prev_yaw, gps_dt, imu_dt;

        nav_msgs::Path gps_path, dr_path, ekf_path;

        geometry_msgs::PoseStamped gps_pose, dr_pose, ekf_pose;

        ros::Time gps_current_time, gps_prev_time, imu_current_time, imu_prev_time;

        utm gps_utm;
        utm dr_utm;
        utm vehicle_utm;
        geometry_msgs::Pose vehicle_prior;

        lanelet::GPSPoint current_pos;
};

#endif