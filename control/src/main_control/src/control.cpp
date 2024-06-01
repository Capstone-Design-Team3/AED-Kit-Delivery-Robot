#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
//#include <autoku_msgs/Gnss.h>
#include <vector>
#include <cmath>
double t_x,t_y;
double gps_x,gps_y;
double L = 0.8 ; //wheelbase
double yaw;
int zone;
bool northp;
double target_steering;

void point_CallBack(const std_msgs::Float32MultiArray::ConstPtr& msg){
  
  double t_x = msg->data[0];
  double t_y = msg->data[1];
  // double temp_long = msg->data[0];
  // double temp_lati = msg->data[1];
  // GeographicLib::UTMUPS::Forward(temp_long, temp_lati, zone, northp, t_x, t_y);
}
//const autoku_msgs::Gnss::ConstPtr& msg
void gps_CallBack(const std_msgs::Float32MultiArray::ConstPtr& msg){
  {
    double gps_x = msg->data[0];//x
    double gps_y = msg->data[1];//y
    // double temp_long = msg->data[0];//longitude;
    // double temp_lati = msg->data[1];//latitude;
    // GeographicLib::UTMUPS::Forward(temp_long, temp_lati, zone, northp, gps_x, gps_y);
    double heading = msg->data[2] + M_PI;//heading; // 0 ~ 2PI
    double dx = t_x - gps_x;
    double dy = t_y - gps_y;
    double distance_to_target = sqrt(dx * dx + dy * dy);

    double angle_to_target = atan2(dy, dx) + M_PI; // 0 ~ 2PI 
    double alpha = heading - angle_to_target ; //Heading 기준과 angle_to_target 기준을 맞춰야함 

    target_steering = atan2(2 * L * sin(alpha), distance_to_target);
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "control");
  ros::NodeHandle nh; //노드핸들러 객체
  
  ros::Publisher target_angle_pub = nh.advertise<std_msgs::Float32>("/target_angle", 10);
  ros::Publisher target_speed_pub = nh.advertise<std_msgs::Float32>("/target_speed", 10);

  ros::Subscriber target_point_sub = nh.subscribe("/next_node", 10, point_CallBack);
  ros::Subscriber gps_sub = nh.subscribe("/kalman_pose",10, gps_CallBack);
  std_msgs::Float32 angle_msg;
  std_msgs::Float32 speed_msg;
  
  while(ros::ok()){
    angle_msg.data = target_steering;
    speed_msg.data = 5;
    target_angle_pub.publish(angle_msg);
    target_speed_pub.publish(speed_msg);
    ros::spinOnce();
  }
  

  
  return 0;
}