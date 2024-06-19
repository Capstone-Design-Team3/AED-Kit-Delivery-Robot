#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include "main_control/Gnss.h"
#include <vector>
#include <cmath>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_projection/UTM.h>

using namespace lanelet;
double t_x,t_y,c_x,c_y,c2_x,c2_y;
double gps_x,gps_y;
double L = 4.377470994 ; //wheelbase
double yaw;
int zone;
bool northp;
double target_steering;
double target_speed = 5;
lanelet::projection::UtmProjector projector(lanelet::Origin({37.5418003, 127.07848369999999}));
lanelet::GPSPoint pathPoint;
lanelet::GPSPoint currentPoint;
lanelet::GPSPoint current2Point;
lanelet::GPSPoint gpsPoint;
lanelet::BasicPoint3d pathPoint_utm;
lanelet::BasicPoint3d gpsPoint_utm;
lanelet::BasicPoint3d currentPoint_utm;
lanelet::BasicPoint3d current2Point_utm;

double get_error(double x1, double y1, double x2, double y2, double x0, double y0) {
    // 직선의 방정식 계수 계산
    double A = y2 - y1;
    double B = -(x2 - x1);
    double C = x2 * y1 - y2 * x1;
    
    // 점과 직선 사이의 거리 계산
    double numerator = std::abs(A * x0 + B * y0 + C);
    double denominator = std::sqrt(A * A + B * B);
    
    double distance = numerator / denominator;
    return distance;
}

void point_CallBack(const std_msgs::Float32MultiArray::ConstPtr& msg){
  
  pathPoint.lat = msg->data[0];
  pathPoint.lon = msg->data[1];
  pathPoint_utm = projector.forward(pathPoint);
  t_x = pathPoint_utm.x();
  t_y = pathPoint_utm.y();
  //std::cout<<"tx:" << t_x << "ty:" << t_y <<std::endl;
}
void stop_CallBack(const std_msgs::Bool::ConstPtr& msg){
 bool stop = msg->data;
 if(stop == true){
  target_speed = 0;
 }
 else{
  target_speed = 5;
 }
}



void gps_CallBack(const main_control::Gnss::ConstPtr& msg){
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
    double alpha = heading - angle_to_target  ; //Heading 기준과 angle_to_target 기준을 맞춰야함 
    //std::cout << "heading:" << heading <<"angle_to_target"<< angle_to_target<<std::endl;
    target_steering = 0.6*(atan2(2 * L * sin(alpha), distance_to_target))*180/M_PI;

    double cross_error = get_error(c_x,c_y,c2_x,c2_y,gps_x,gps_y);
    double heading_error = heading - atan2((c2_y-c_y),(c2_x-c_x));
    std::cout << "cross_error: " << cross_error << std::endl;
    std::cout << "heading_error: " << heading_error << std::endl;
}

void record_CallBack(const std_msgs::Float32MultiArray::ConstPtr& msg){
  currentPoint.lat = msg->data[0];
  currentPoint.lon = msg->data[1];
  currentPoint_utm = projector.forward(currentPoint);
  c_x = currentPoint_utm.x();
  c_y = currentPoint_utm.y();
}
void record2_CallBack(const std_msgs::Float32MultiArray::ConstPtr& msg){
  current2Point.lat = msg->data[0];
  current2Point.lon = msg->data[1];
  current2Point_utm = projector.forward(current2Point);
  c2_x = current2Point_utm.x();
  c2_y = current2Point_utm.y();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "control");
  ros::NodeHandle nh; //노드핸들러 객체
  
  ros::Publisher target_angle_pub = nh.advertise<std_msgs::Float32>("/target_angle", 10);
  ros::Publisher target_speed_pub = nh.advertise<std_msgs::Float32>("/target_speed", 10);

  ros::Subscriber target_point_sub = nh.subscribe("/next_node", 3, point_CallBack);
  ros::Subscriber gps_sub = nh.subscribe("/kalman_pose",3, gps_CallBack);
  ros::Subscriber stop_sub = nh.subscribe("/stop_decision",3, stop_CallBack);
  ros::Subscriber current_sub = nh.subscribe("/current_node",3, record_CallBack);
  ros::Subscriber current2_sub = nh.subscribe("/current2_node",3, record2_CallBack);
  std_msgs::Float32 angle_msg;
  std_msgs::Float32 speed_msg;
  


  ros::Rate loop_rate(50);
  
  while(ros::ok()){
     ros::spinOnce();
    angle_msg.data = target_steering;
    speed_msg.data = target_speed;
    target_angle_pub.publish(angle_msg);
    target_speed_pub.publish(speed_msg);
    //std::cout << "UTM X: " << t_x<< ", UTM Y: " << t_y << std::endl;
    //std::cout << "GPS X: " << gps_x << ", GPS Y: " << gps_y << std::endl;
    //std::cout << "Target_Steering:" << target_steering << std::endl;
    

   loop_rate.sleep();
  }
  

  
  return 0;
}