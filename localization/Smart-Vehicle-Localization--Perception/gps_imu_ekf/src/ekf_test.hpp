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
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include "gps_imu_ekf/Gnss.h"

using namespace Eigen;

const int N = 3, M = 2; //N =  state (x, y, yaw), M = Measure (x, y)

MatrixXd Q(N,N), I(N,N), F_jacob(N,N), P_post(N,N), P_prior(N,N), K(N,M), R(M,M), H_jacob(M,N);
VectorXd f(N), x_post(N), x_prior(N), z(M), h(M), f_dr(N); 

lanelet::projection::UtmProjector projection(lanelet::Origin({37.541759, 127.078433}));

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
        ros::Publisher box_pub;
        ros::Publisher ekf_path_pub;
        ros::Publisher yaw_bias_pub;
        ros::Publisher dr_path_pub;
        ros::Subscriber m_gps_sub;
        ros::Subscriber m_imu_sub;
        ros::Subscriber m_vehicle_speed_sub;

    public:
        ExtendedKalmanFilter();
        ~ExtendedKalmanFilter();

        void init();

        void state_init();

        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

        void speedCallback(const std_msgs::Float32::ConstPtr& msg);

        void DeadReckoning();

        void EKF();

        VectorXd f_k(VectorXd x_post);

        void Visualization(geometry_msgs::PoseStamped gps_pose , geometry_msgs::PoseStamped dr_pose, geometry_msgs::PoseStamped ekf_pose);

        void visualizeHeading(geometry_msgs::PoseStamped ekf_pose, jsk_recognition_msgs::BoundingBox car_model);

        void publishPose();

        double getVehicleSpeed(utm utm);

        bool gps_input, state_init_check, measure_check,first_time;

        double yaw_rate, yaw, prev_yaw, dt, yaw_bias, imu_dt, gps_dt,h_time, PoCo;

        int prediction_count, gps_count, ekf_count, yaw_bias_count;

        tf::TransformBroadcaster tfcaster;

        nav_msgs::Path gps_path, ekf_path, dr_path;

        std_msgs::Float64 yaw_bias_data;

        visualization_msgs::Marker marker;

        jsk_recognition_msgs::BoundingBox car_model;

        geometry_msgs::PoseStamped gps_pose, ekf_pose, dr_pose;

        ros::Time previous_time, current_time,gps_current_time,gps_prev_time,flow_time;

        utm gps_utm;
        utm vehicle_utm;
        utm dr_utm;
        gps_imu_ekf::Gnss k_pose;

        lanelet::GPSPoint current_pos;
};

#endif