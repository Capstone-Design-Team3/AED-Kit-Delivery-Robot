#include <iostream>
#include <cmath>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;

class PassThroughFilter {
  private:
    ros::NodeHandle nh;
    ros::Publisher pub_pcl2;
    ros::Subscriber sub_scan_pcl;
    sensor_msgs::LaserScan laser_scan_pcl;
    sensor_msgs::PointCloud2 laser_scan_pcl2;

  public:
    PassThroughFilter();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_pcl);
    void pass_through_filter();
    sensor_msgs::PointCloud2 laser2cloudmsg(sensor_msgs::LaserScan laser);
    void Run();
    void Publish();
};

PassThroughFilter::PassThroughFilter() {
    pub_pcl2 = nh.advertise<sensor_msgs::PointCloud2>("/pass_through_filter_pcl2", 100);
    sub_scan_pcl = nh.subscribe("/scan", 100, &PassThroughFilter::scanCallback, this);
}

void PassThroughFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_pcl) {
    laser_scan_pcl = *scan_pcl;
}

void PassThroughFilter::pass_through_filter() {
    // // camera 화각으로 설정
    double angle = 180/90;
    double min_angle = -M_PI/angle;
    double max_angle = M_PI/angle;
    // double min_angle = -2.1;
    // double max_angle = 2.1;

    // // for (int i = 0; i < laser_scan_pcl.ranges.size(); ++i) {
    // //     double angle = laser_scan_pcl.angle_min + i * laser_scan_pcl.angle_increment;
    // //     if (!(angle < min_angle || angle > max_angle)) {
    // //         laser_scan_pcl.ranges[i] = std::numeric_limits<double>::infinity();
    // //     }
    // // }

    // // TODO: tunning
    // // 인지하는 거리로 설정
    // double min_distance = 0.0;
    // double max_distance = 25.0;

    // for (int i = 0; i < laser_scan_pcl.ranges.size(); ++i) {
    //     double angle = laser_scan_pcl.angle_min + i * laser_scan_pcl.angle_increment;
    //     double range = laser_scan_pcl.ranges[i];
    //     if (!(angle < min_angle || angle > max_angle) || (range < min_distance || range > max_distance)) {
    //         laser_scan_pcl.ranges[i] = std::numeric_limits<double>::infinity();
    //     }
    // }

    double cx = 0.0; // Center x
    double cy = 0.0; // Center y
    double width = 10.0; // Width of the rectangle
    double height = 2.0; // Height of the rectangle
    
    double x_min = cx - width / 2.0;
    double x_max = cx + width / 2.0;
    double y_min = cy - height / 2.0;
    double y_max = cy + height / 2.0;

    for (int i = 0; i < laser_scan_pcl.ranges.size(); ++i) {
        double angle = laser_scan_pcl.angle_min + i * laser_scan_pcl.angle_increment;
        double range = laser_scan_pcl.ranges[i];

        // Convert polar coordinates to Cartesian coordinates
        double x = range * cos(angle);
        double y = range * sin(angle);

        // Check if the point is inside the rectangular ROI
        if (!(x >= x_min && x <= x_max && y >= y_min && y <= y_max) || !(angle < min_angle || angle > max_angle)) {
            laser_scan_pcl.ranges[i] = std::numeric_limits<double>::infinity();
        }
    }
}

sensor_msgs::PointCloud2 PassThroughFilter::laser2cloudmsg(sensor_msgs::LaserScan laser) {
    static laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 pc2_dst;
    projector.projectLaser(laser, pc2_dst,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
    // pc2_dst.header.frame_id = "map";
    pc2_dst.header.frame_id = "laser_frame";

    return pc2_dst;
}

void PassThroughFilter::Run() {
    pass_through_filter();
    laser_scan_pcl2 = laser2cloudmsg(laser_scan_pcl);
    Publish();
}

void PassThroughFilter::Publish() {
    pub_pcl2.publish(laser_scan_pcl2);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pass_through_filter");
    PassThroughFilter pass_through_filter;
    ros::Rate loop_rate(30);    

    while(ros::ok()) {
        pass_through_filter.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}