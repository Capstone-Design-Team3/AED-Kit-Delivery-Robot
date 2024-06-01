#include <iostream>
#include <cmath>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

using namespace std;

class StatisticalOutlierRemoval {
  private:
    ros::NodeHandle nh;
    ros::Publisher pub_pcl;
    ros::Subscriber sub_scan_pcl;
    sensor_msgs::PointCloud2 laser_scan_pcl;
    sensor_msgs::PointCloud2 statistical_outlier_removal_pcl;

  public:
    StatisticalOutlierRemoval();
    void psfCallback(const sensor_msgs::PointCloud2::ConstPtr& scan_pcl);
    sensor_msgs::PointCloud2 statistical_outlier_removal();
    void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color);
    void colorPointCloud(pcl::PCLPointCloud2::Ptr& cloud);
    void cloud2ptr();
    void Run();
    void Publish();
};

StatisticalOutlierRemoval::StatisticalOutlierRemoval() {
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/statistical_outlier_removal_pcl2", 100);
    sub_scan_pcl = nh.subscribe("/pass_through_filter_pcl2", 100, &StatisticalOutlierRemoval::psfCallback, this);
}

void StatisticalOutlierRemoval::psfCallback(const sensor_msgs::PointCloud2::ConstPtr& scan_pcl) {
    laser_scan_pcl = *scan_pcl;
}

sensor_msgs::PointCloud2 StatisticalOutlierRemoval::statistical_outlier_removal() {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(laser_scan_pcl, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    
    // TODO: tunning
    int num_neigbor_points = 15;  // 인접한 점들의 개수 설정
    double std_multiplier = 2.0;  // 이상점으로 처리할 표준편차의 배수 설정
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(num_neigbor_points);
    sor.setStddevMulThresh(std_multiplier);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_filtered);

    // color 입히기
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    // colorize(*cloud_filtered, *out_colored, {0, 255, 0});
    // pcl::visualization::PCLVisualizer viewer1("filtered");
    // viewer1.addPointCloud<pcl::PointXYZRGB>(out_colored, "filtered_green");
    // viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "filtered_green");
    // while (!viewer1.wasStopped()) {
    //     viewer1.spinOnce();
    // }

    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_cloud_msg);
    filtered_cloud_msg.header.frame_id = "laser_frame";

    return filtered_cloud_msg;
}

void StatisticalOutlierRemoval::colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {

    int N = pc.points.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int i = 0; i < N; ++i) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

void StatisticalOutlierRemoval::colorPointCloud(pcl::PCLPointCloud2::Ptr& cloud) {
    // Check if RGB field already exists
    bool has_rgb = false;
    for (size_t i = 0; i < cloud->fields.size(); ++i) {
        if (cloud->fields[i].name == "rgb") {
            has_rgb = true;
            break;
        }
    }

    // If RGB field doesn't exist, add it
    if (!has_rgb) {
        cloud->fields.resize(4);
        cloud->fields[3].name = "rgb";
        cloud->fields[3].offset = cloud->fields[0].offset + sizeof(float);
        cloud->fields[3].count = 1;
        cloud->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    }

    // Fill in RGB color information
    for (size_t i = 0; i < cloud->width * cloud->height; ++i)
    {
        uint32_t rgb = (static_cast<uint32_t>(255) << 16 | static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(0));
        std::memcpy(&cloud->data[i * cloud->point_step + cloud->fields[3].offset], &rgb, sizeof(float));
    }
}

void StatisticalOutlierRemoval::cloud2ptr() {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(statistical_outlier_removal_pcl, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);
    pcl::PCLPointCloud2::Ptr cloud_ptr(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(cloud, *cloud_ptr);
    colorPointCloud(cloud_ptr);
}

void StatisticalOutlierRemoval::Run() {
    if(laser_scan_pcl.data.size() > 0) {
        statistical_outlier_removal_pcl = statistical_outlier_removal();
        // cloud2ptr();
    }
    Publish();
}

void StatisticalOutlierRemoval::Publish() {
    pub_pcl.publish(statistical_outlier_removal_pcl);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "statistical_outlier_removal");
    StatisticalOutlierRemoval statistical_outlier_removal;
    ros::Rate loop_rate(30);    

    while(ros::ok()) {
        statistical_outlier_removal.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}