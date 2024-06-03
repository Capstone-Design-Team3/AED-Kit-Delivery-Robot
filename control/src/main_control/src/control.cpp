#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "main_control/Gnss.h"
#include <vector>
#include <cmath>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_projection/UTM.h>

using namespace lanelet;
double t_x,t_y;
double gps_x,gps_y;
double L = 4.377470994; //wheelbase
double yaw;
int zone;
bool northp;
double target_steering;
lanelet::projection::UtmProjector projector(lanelet::Origin({37.541759, 127.078433}));
lanelet::GPSPoint pathPoint;
lanelet::GPSPoint gpsPoint;
lanelet::BasicPoint3d pathPoint_utm;
lanelet::BasicPoint3d gpsPoint_utm;



void point_CallBack(const std_msgs::Float32MultiArray::ConstPtr& msg){
  
  pathPoint.lon = msg->data[0];
  pathPoint.lat = msg->data[1];
  pathPoint_utm = projector.forward(pathPoint);
  t_x = pathPoint_utm.x();
  t_y = pathPoint_utm.y();
  std::cout<<"tx:" << t_x << "ty:" << t_y <<std::endl;
}

void gps_CallBack(const main_control::Gnss::ConstPtr& msg){
  {
    gpsPoint.lon = msg->longitude;//x
    gpsPoint.lat = msg->latitude;//y
    gpsPoint_utm = projector.forward(gpsPoint);
    gps_x = gpsPoint_utm.x();
    gps_y = gpsPoint_utm.y();


    double heading = msg->heading;//heading; // -PI ~ PI
    double dx = t_x - gps_x;
    double dy = t_y - gps_y;
    double distance_to_target = sqrt(dx * dx + dy * dy);

    double angle_to_target = atan2(dy, dx) ; // -PI ~ PI 
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
    speed_msg.data = 0;
    target_angle_pub.publish(angle_msg);
    target_speed_pub.publish(speed_msg);
    std::cout << "UTM X: " << t_x<< ", UTM Y: " << t_y << std::endl;
    std::cout << "GPS X: " << gps_x << ", GPS Y: " << gps_y << std::endl;
    std::cout << "Target_Steering:" << target_steering << std::endl;
    std::cout << "heading:" << heading <<"angle_to_target:"<<angle_to_target<< std::endl;
    ros::spinOnce();
  }
  

  
  return 0;
}
