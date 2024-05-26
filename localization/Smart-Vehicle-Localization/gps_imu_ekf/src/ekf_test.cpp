#include "ekf_test.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter(){
    m_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/ublox_gps/fix", 100, &ExtendedKalmanFilter::gpsCallback, this);
    m_imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data",1,&ExtendedKalmanFilter::imuCallback,this);
    m_vehicle_speed_sub = nh.subscribe<std_msgs::Float32>("/velocity",1,&ExtendedKalmanFilter::speedCallback, this);
    
    dr_path_pub = nh.advertise<nav_msgs::Path>("dr_path", 1);
    gps_path_pub = nh.advertise<nav_msgs::Path>("gps_path", 1);
    ekf_path_pub = nh.advertise<nav_msgs::Path>("ekf_path", 1);
    m_visual_pub = nh.advertise<geometry_msgs::PoseStamped>("heading",1);
    box_pub = nh.advertise<jsk_recognition_msgs::BoundingBox>("car_model",1);

    m_pose_pub = nh.advertise<gps_imu_ekf::Gnss>("kalman_pose",100);
    yaw_bias_pub = nh.advertise<std_msgs::Float64>("/yaw_bias",1);

    gps_input = false, state_init_check = false,first_time=false;
}

ExtendedKalmanFilter::~ExtendedKalmanFilter(){}

void ExtendedKalmanFilter::init(){
    gps_utm.x = 0;
    gps_utm.y = 0;
     
    dr_utm.x = 0;
    dr_utm.y = 0;

    vehicle_utm.x = 0;
    vehicle_utm.y = 0;

    prev_yaw = 0;
    yaw_bias = 0;
    yaw_bias_count = 1;
    
   
    dt = 1.0 / 80;
    gps_dt = 1.0;
    vehicle_utm.velocity = 0;
    prediction_count = 0;
}

void ExtendedKalmanFilter::state_init(){
    if(vehicle_utm.velocity >=0.4 &&gps_input && !state_init_check){  //vehicle_utm.velocity>0 &&없어서 빙글 빙글 돌았던거
        state_init_check = true;
        measure_check = false;
        prev_yaw = atan2((gps_utm.y-gps_utm.prev_y),(gps_utm.x-gps_utm.prev_x)); //이거때문에 초반 yaw
        std::cout <<"gps_utm.prev_x: "<<gps_utm.prev_x<<"\n"<<std::endl;
        std::cout <<"gps_utm.prev_y : "<<gps_utm.prev_y<<"\n"<<std::endl;
        std::cout <<"gps_utm.x: "<<gps_utm.x<<"\n"<<std::endl;
         std::cout <<"gps_utm.y: "<<gps_utm.y<<"\n"<<std::endl;
         std::cout <<"prev_yaw : "<<prev_yaw<<"\n"<<std::endl;

        dr_utm.prev_x = gps_utm.x;
        dr_utm.prev_y = gps_utm.y;
        
        //초기값
        x_post(0) = gps_utm.x;
        x_post(1) = gps_utm.y;
        x_post(2) = prev_yaw;
        P_post << 1000, 0, 0,
                    0, 1000, 0,
                    0, 0, 1000;  //클수록 gps에 의존
    }
}

// double ExtendedKalmanFilter::getVehicleSpeed(utm utm){
//     //추가
//    if(h_time>20)
//     return std::sqrt(std::pow(utm.x - utm.prev_x,2)+std::pow(utm.y - utm.prev_y,2))/gps_dt;
//    else
//     return 0; 
// }

void ExtendedKalmanFilter::DeadReckoning(){
    if(state_init_check ){
        dr_utm.x = dr_utm.prev_x + (vehicle_utm.velocity * dt * cos(vehicle_utm.yaw));
        dr_utm.y = dr_utm.prev_y + (vehicle_utm.velocity * dt * sin(vehicle_utm.yaw));
        dr_utm.yaw = dr_utm.prev_yaw + dt * yaw_rate;
        dr_utm.prev_x = dr_utm.x;
        dr_utm.prev_y = dr_utm.y;
    }
}


void ExtendedKalmanFilter::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{   
    
    lanelet::projection::UtmProjector projection(lanelet::Origin({ 37.5422589, 127.0793964}));
 
    measure_check = true;
    
     
    gps_current_time = msg->header.stamp;
    gps_dt = (gps_current_time - gps_prev_time).toSec(); // gps_dt 계산
    gps_prev_time = gps_current_time;



    current_pos.lat = msg -> latitude;
    current_pos.lon = msg -> longitude;
    if(!gps_input){ // 처음 한번만
        gps_utm.prev_x = projection.forward(current_pos).x();
        gps_utm.prev_y = projection.forward(current_pos).y();
        gps_input = true;
    }
    else{ // 계속 여기 걸림
        gps_utm.prev_x = gps_utm.x;
        gps_utm.prev_y = gps_utm.y;
    }

    gps_utm.x = projection.forward(current_pos).x();
    gps_utm.y = projection.forward(current_pos).y();

    if(!state_init_check){
        k_pose.latitude = msg->latitude;
        k_pose.longitude = msg->longitude;
    }
    //vehicle_utm.velocity = getVehicleSpeed(gps_utm);

    

}

void ExtendedKalmanFilter::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    // if(vehicle_utm.velocity ==0 && yaw_bias_count < INT_MAX ){
    //     yaw_bias+=(msg->angular_velocity.z)/yaw_bias_count;
    //     yaw_bias_count++;
    //     std :: cout <<"Yaw bias 체크 중"<<yaw_bias_count<<"\n"<<std :: endl;
    // }
    // else if(vehicle_utm.velocity <0){
    //     yaw_bias = 0;
    //     yaw_bias_count = 1;
    // }

    yaw_rate = msg -> angular_velocity.z - yaw_bias;
    yaw_bias_data.data = yaw_rate;
    yaw_bias_pub.publish(yaw_bias_data);
}

void ExtendedKalmanFilter::speedCallback(const std_msgs::Float32::ConstPtr& msg){
    vehicle_utm.velocity = msg->data / 3.6;
}

VectorXd ExtendedKalmanFilter::f_k(VectorXd x_post){
    VectorXd fk(N); 
    fk << (x_post(0) + vehicle_utm.velocity * dt * cos(x_post(2))),
        (x_post(1) + vehicle_utm.velocity * dt * sin(x_post(2))),
        (x_post(2) + yaw_rate * dt);
    return fk;
}

void ExtendedKalmanFilter::EKF(){
    current_time = ros::Time::now();
    dt = (current_time - previous_time).toSec();

    //코드추가
    if(first_time == false){
        flow_time = current_time;
        first_time = true;
    }
    h_time = (current_time - flow_time).toSec();

    //std::cout<<h_time<<std :: endl;
   //

     
    if(vehicle_utm.velocity == 0){
        Q << 0.01, 0.00, 0.00,      //값 높이면 측정값 비중 증가
             0.00, 0.01, 0.00,
             0.00, 0.00, 0.01;

        R << 0.01, 0.0,               //값 높이면 센서값 비중 증가
             0.0, 0.01;
    }
    else{
        Q << 0.1, 0.0, 0.0,          //값 높이면 측정값 비중 증가
             0.0, 0.1, 0.0,
             0.0, 0.0, 0.1;

        R << 0.01, 0.0,               //값 높이면 IMU 센서값 비중 증가
             0.0, 0.01;
    }
    
    if(state_init_check){

        I.setIdentity();

        H_jacob << 1.0, 0.0, 0.0,
             0.0, 1.0, 0.0;

        z << gps_utm.x,
            gps_utm.y;

        F_jacob << 1.0, 0.0, -1*vehicle_utm.velocity*dt*sin(vehicle_utm.yaw), 
             0.0, 1.0, vehicle_utm.velocity*dt*cos(vehicle_utm.yaw),
             0.0, 0.0, 1.0;

        x_prior = f_k(x_post);
        f_dr = f_k(x_post);

        P_prior = F_jacob * P_post * F_jacob.transpose() + Q;

        x_post = x_prior;

        P_post = P_prior;

        prediction_count++;

        if(measure_check){
            h << x_prior(0),
                x_prior(1);

            K = P_prior * H_jacob.transpose()*(H_jacob*P_prior*H_jacob.transpose()+R).inverse();
            
            x_post = x_prior + K*(z-h);

            P_post = (I - K * H_jacob) * P_prior;

            prediction_count = 0;

            measure_check = false;
        }

        vehicle_utm.x = x_post(0);
        vehicle_utm.y = x_post(1);
        vehicle_utm.yaw = x_post(2);
    }
    previous_time = current_time;
}

void ExtendedKalmanFilter::publishPose(){
    lanelet::BasicPoint3d utm_point(vehicle_utm.x, vehicle_utm.y, 0);
    std::cout <<"utm_point : "<<utm_point<<"\n"<<std::endl;
    lanelet::GPSPoint gps_point = projection.reverse(utm_point);

    if (vehicle_utm.yaw > M_PI){
        vehicle_utm.yaw -= 2.0 * M_PI;
    }
    else if (vehicle_utm.yaw < -M_PI){
        vehicle_utm.yaw += 2.0 * M_PI;
    }

    k_pose.header.frame_id = "world";

    k_pose.latitude = gps_point.lat;
    k_pose.longitude = gps_point.lon;

    k_pose.heading = vehicle_utm.yaw;

    m_pose_pub.publish(k_pose);
       std::cout <<"k_pose : "<<k_pose<<"\n"<<std::endl;
}

void ExtendedKalmanFilter::Visualization(geometry_msgs::PoseStamped gps_pose, geometry_msgs::PoseStamped dr_pose, geometry_msgs::PoseStamped ekf_pose){

    gps_pose.header.frame_id = "map";
    gps_pose.header.stamp = ros::Time::now();
    gps_pose.pose.position.x = gps_utm.x;
    gps_pose.pose.position.y = gps_utm.y;
    gps_pose.pose.position.z = 0;
    gps_pose.pose.orientation.x = 0.0;
    gps_pose.pose.orientation.y = 0.0;
    gps_pose.pose.orientation.z = 0.0;
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
    ekf_pose.pose.orientation.z = 0.0;
    ekf_pose.pose.orientation.w = 1.0;
    ekf_path.poses.push_back(ekf_pose);
    ekf_path.header.stamp = ros::Time::now();
    ekf_path.header.frame_id = "map";
    ekf_path_pub.publish(ekf_path);
}

void ExtendedKalmanFilter::visualizeHeading(geometry_msgs::PoseStamped ekf_pose, jsk_recognition_msgs::BoundingBox car_model){

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(vehicle_utm.x, vehicle_utm.y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, vehicle_utm.yaw);
    transform.setRotation(q);
    tfcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "my_car"));

    ekf_pose.header.frame_id = "map";
    ekf_pose.header.stamp = ros::Time::now();
    ekf_pose.pose.position.x = vehicle_utm.x;
    ekf_pose.pose.position.y = vehicle_utm.y;
    ekf_pose.pose.position.z = 0;
    ekf_pose.pose.orientation = tf::createQuaternionMsgFromYaw(vehicle_utm.yaw);

    car_model.header.frame_id = "map";
    car_model.header.stamp = ros::Time::now();
    car_model.dimensions.x = 3.0;
    car_model.dimensions.y = 1.5;
    car_model.dimensions.z = 1.0;
    car_model.pose.position.x = vehicle_utm.x;
    car_model.pose.position.y = vehicle_utm.y;
    car_model.pose.orientation = tf::createQuaternionMsgFromYaw(vehicle_utm.yaw);

    m_visual_pub.publish(ekf_pose);
    box_pub.publish(car_model);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "kalman_test");

    ExtendedKalmanFilter ExtendedKalmanFilter;

    ExtendedKalmanFilter.init();

    ros::Rate loop_rate(80); // loop 주기 80HZ

    while (ros::ok()){

        ExtendedKalmanFilter.state_init();
        ExtendedKalmanFilter.EKF();
        ExtendedKalmanFilter.DeadReckoning();
        ExtendedKalmanFilter.Visualization(ExtendedKalmanFilter.gps_pose, ExtendedKalmanFilter.dr_pose, ExtendedKalmanFilter.ekf_pose);
        ExtendedKalmanFilter.visualizeHeading(ExtendedKalmanFilter.ekf_pose, ExtendedKalmanFilter.car_model);
        ExtendedKalmanFilter.publishPose();

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}