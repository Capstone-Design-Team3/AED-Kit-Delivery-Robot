#include "gps_imu_ekf.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter(){
    m_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/ublox_gps/fix", 100, &ExtendedKalmanFilter::gpsCallback, this);
    m_imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data",1,&ExtendedKalmanFilter::imuCallback,this);
    
    gps_path_pub = nh.advertise<nav_msgs::Path>("gps_path", 1);
    dr_path_pub = nh.advertise<nav_msgs::Path>("dr_path", 1);
    ekf_path_pub = nh.advertise<nav_msgs::Path>("ekf_path", 1);
    m_pose_pub = nh.advertise<geometry_msgs::PoseStamped>( "kalman_pose", 100);

    gps_input = false, state_init_check = false;
}

ExtendedKalmanFilter::~ExtendedKalmanFilter(){}

void ExtendedKalmanFilter::init(){
    gps_utm.x = 0;
    gps_utm.y = 0;

    dr_utm.x = 0;
    dr_utm.y = 0;

    vehicle_utm.x = 0;
    vehicle_utm.y = 0;
    vehicle_utm.velocity = 0;

    vehicle_prior.position.x = 0;
    vehicle_prior.position.y = 0;

    prev_yaw = 0;

    gps_dt = 1;
    imu_dt = 0.02;
}

void ExtendedKalmanFilter::state_init(){
    if(vehicle_utm.velocity>0 && gps_input && !state_init_check){
        state_init_check = true;
        dr_utm.prev_x = gps_utm.x;
        dr_utm.prev_y = gps_utm.y;
        prev_yaw = atan2((gps_utm.y-gps_utm.prev_y),(gps_utm.x-gps_utm.prev_x));
    }
}

double ExtendedKalmanFilter::getVehicleSpeed(utm utm){
    return std::sqrt(std::pow(utm.x - utm.prev_x,2)+std::pow(utm.y - utm.prev_y,2))/gps_dt;
}

void ExtendedKalmanFilter::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    lanelet::projection::UtmProjector projection(lanelet::Origin({37.541759, 127.078433}));

    gps_current_time = msg->header.stamp;
    gps_dt = (gps_current_time - gps_prev_time).toSec();
    gps_prev_time = gps_current_time;

    current_pos.lat = msg -> latitude;
    current_pos.lon = msg -> longitude;
    if(!gps_input){
        gps_utm.prev_x = projection.forward(current_pos).x();
        gps_utm.prev_y = projection.forward(current_pos).y();
        gps_input = true;
    }
    else{
        gps_utm.prev_x = gps_utm.x;
        gps_utm.prev_y = gps_utm.y;
    }

    gps_utm.x = projection.forward(current_pos).x();
    gps_utm.y = projection.forward(current_pos).y();

    vehicle_prior.position.x = gps_utm.x;
    vehicle_prior.position.y = gps_utm.y;

    //vehicle_utm.velocity = getVehicleSpeed(gps_utm);
}

void ExtendedKalmanFilter::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    imu_current_time = msg->header.stamp;
    imu_dt = (imu_current_time - imu_prev_time).toSec();
    imu_prev_time = imu_current_time;
    
    yaw_rate = msg -> angular_velocity.z;

    if(state_init_check){
        yaw = prev_yaw + yaw_rate*imu_dt;

        vehicle_utm.yaw = yaw;

        vehicle_prior.orientation.z = vehicle_utm.yaw;
        prev_yaw = vehicle_utm.yaw;
    }
}

void ExtendedKalmanFilter::DeadReckoning(){
    if(state_init_check){
        dr_utm.x = dr_utm.prev_x + (vehicle_utm.velocity * imu_dt * cos(vehicle_utm.yaw));
        dr_utm.y = dr_utm.prev_y + (vehicle_utm.velocity * imu_dt * sin(vehicle_utm.yaw));
        dr_utm.yaw = dr_utm.prev_yaw + imu_dt * yaw_rate;
        dr_utm.prev_x = dr_utm.x;
        dr_utm.prev_y = dr_utm.y;
    }
}

void ExtendedKalmanFilter::EKF(){
    if(state_init_check){
        Q << 0.1, 0.0, 0.0,          //값 높이면 측정값 비중 증가
             0.0, 0.1, 0.0,
             0.0, 0.0, 0.1;

        R << 0.01, 0.0,               //값 높이면 센서값 비중 증가
             0.0, 0.01;

        I.setIdentity();

        H << 1.0, 0.0, 0.0,
             0.0, 1.0, 0.0;

        F << 1.0, 0.0, -1*vehicle_utm.velocity*imu_dt*sin(vehicle_utm.yaw), 
             0.0, 1.0, vehicle_utm.velocity*imu_dt*cos(vehicle_utm.yaw),
             0.0, 0.0, 1.0;

        f << dr_utm.x,
             dr_utm.y,
             dr_utm.yaw;
                //m_vehicle_utm.yaw + 0.1*md_yawrate;

        x_predict << f(0),
                     f(1),
                     f(2);
        P_predict = F*P_post*F.transpose() + Q;
 
        // ////////////칼만 이득 계산//////////////////////////////////////
        K = P_predict*H.transpose()*(H*P_predict*H.transpose()+R).inverse();

        // ////////////추정값 계산/////////////////////////////////////////
        
        z << gps_utm.x,
             gps_utm.y;

        h << x_predict(0),
             x_predict(1);
        

        x_prior = x_predict + K*(z-h);

        // ////////////오차 공분산 계산/////////////////////////////////////
        P_prior = (I - K * H) * P_predict;

        vehicle_utm.x = x_prior(0);
        vehicle_utm.y = x_prior(1);

        vehicle_prior.position.x = vehicle_utm.x;
        vehicle_prior.position.y = vehicle_utm.y;        

        x_post = x_prior;
        P_post = P_prior;

    }
}

void ExtendedKalmanFilter::publishPose(){
    m_pose_pub.publish(vehicle_prior);
}

void ExtendedKalmanFilter::Visualization(geometry_msgs::PoseStamped gps_pose, geometry_msgs::PoseStamped dr_pose, geometry_msgs::PoseStamped ekf_pose){

    gps_pose.header.frame_id = "map";
    gps_pose.header.stamp = ros::Time::now();
    gps_pose.pose.position.x = gps_utm.x;
    gps_pose.pose.position.y = gps_utm.y;
    gps_pose.pose.position.z = 0;
    gps_pose.pose.orientation.x = 0.0;
    gps_pose.pose.orientation.y = 0.0;
    gps_pose.pose.orientation.z = vehicle_utm.yaw*180/M_PI;
    gps_pose.pose.orientation.w = 1.0;
    gps_path.poses.push_back(gps_pose);
    gps_path.header.stamp = ros::Time::now();
    gps_path.header.frame_id = "map";
    gps_path_pub.publish(gps_path);

    dr_pose.header.frame_id = "map";
    dr_pose.header.stamp = ros::Time::now();
    dr_pose.pose.position.x = dr_utm.x;
    dr_pose.pose.position.y = dr_utm.y;
    dr_pose.pose.position.z = 0;
    dr_pose.pose.orientation.x = 0.0;
    dr_pose.pose.orientation.y = 0.0;
    dr_pose.pose.orientation.z = vehicle_utm.yaw*180/M_PI;
    dr_pose.pose.orientation.w = 1.0;
    dr_path.poses.push_back(dr_pose);
    dr_path.header.stamp = ros::Time::now();
    dr_path.header.frame_id = "map";
    dr_path_pub.publish(dr_path);

    ekf_pose.header.frame_id = "map";
    ekf_pose.header.stamp = ros::Time::now();
    ekf_pose.pose.position.x = vehicle_utm.x;
    ekf_pose.pose.position.y = vehicle_utm.y;
    ekf_pose.pose.position.z = 0;
    ekf_pose.pose.orientation.x = 0.0;
    ekf_pose.pose.orientation.y = 0.0;
    ekf_pose.pose.orientation.z = vehicle_utm.yaw*180/M_PI;
    ekf_pose.pose.orientation.w = 1.0;
    ekf_path.poses.push_back(ekf_pose);
    ekf_path.header.stamp = ros::Time::now();
    ekf_path.header.frame_id = "map";
    ekf_path_pub.publish(ekf_path);

    //ROS_INFO_STREAM("vehicle yaw: "<< vehicle_utm.yaw <<" vehicle utm yaw degree: "<<vehicle_utm.yaw*57);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "kalman");

    ExtendedKalmanFilter ExtendedKalmanFilter;

    ExtendedKalmanFilter.init();

    ros::Rate loop_rate(70);

    while (ros::ok()){

        ExtendedKalmanFilter.state_init();
        ExtendedKalmanFilter.DeadReckoning();
        ExtendedKalmanFilter.EKF();
        ExtendedKalmanFilter.Visualization(ExtendedKalmanFilter.gps_pose, ExtendedKalmanFilter.dr_pose, ExtendedKalmanFilter.ekf_pose);
        ExtendedKalmanFilter.publishPose();

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}