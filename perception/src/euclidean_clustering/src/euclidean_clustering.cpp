#include <iostream>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <lanelet2_projection/UTM.h>
#include <sensor_msgs/Imu.h>
#include <euclidean_clustering/Gnss.h>
#include <euclidean_clustering/ObjectInfoArray.h>
#include <euclidean_clustering/ObjectInfo.h>

using namespace std;

class EuclideanClustering {
  private:
    ros::NodeHandle nh;
    ros::Publisher pub_pcl;
    ros::Subscriber sub_scan_pcl;
    sensor_msgs::PointCloud2 statistical_outlier_removal_pcl;
    sensor_msgs::PointCloud2 euclidean_clustering_pcl;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_clouds;

    // ros::Subscriber sub_gps;
    lanelet::GPSPoint current_pos;
    double vehicle_x, vehicle_y, vehicle_yaw;
    sensor_msgs::NavSatFix gps_info;
    ros::Subscriber sub_gnss;
    euclidean_clustering::Gnss gnss_pose;
    euclidean_clustering::ObjectInfoArray object_info_array;
    ros::Publisher pub_object_info_array;

  public:
    EuclideanClustering();
    void sorCallback(const sensor_msgs::PointCloud2::ConstPtr& scan_pcl);
    // void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
    void GNSSCallback(const euclidean_clustering::Gnss::ConstPtr& gnss_msg);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclidean_clustering();
    void CalcInfo(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& clusters);
    void ObjectInfo(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cluster_clouds);
    std::pair<double, double> CoordinateTranform(double input_x, double input_y);
    void Print();
    void Run();
    void Publish();
};

EuclideanClustering::EuclideanClustering() {
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/euclidean_clustering_pcl2", 100);
    sub_scan_pcl = nh.subscribe("/statistical_outlier_removal_pcl2", 100, &EuclideanClustering::sorCallback, this);
    // sub_gps = nh.subscribe("/ublox_gps/fix", 100, &EuclideanClustering::GPSCallback, this);
    sub_gnss = nh.subscribe("kalman_pose", 100, &EuclideanClustering::GNSSCallback, this);
    pub_object_info_array = nh.advertise<euclidean_clustering::ObjectInfoArray>("/object_info_array", 100);
}

void EuclideanClustering::sorCallback(const sensor_msgs::PointCloud2::ConstPtr& scan_pcl) {
    statistical_outlier_removal_pcl = *scan_pcl;
}

// void EuclideanClustering::GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
//     // gps_info = *gps_msg;
//     lanelet::projection::UtmProjector projection(lanelet::Origin({37.5422589, 127.0793964}));
    
//     current_pos.lat = gps_msg -> latitude;
//     current_pos.lon = gps_msg -> longitude;

//     vehicle_x = projection.forward(current_pos).x();
//     vehicle_y = projection.forward(current_pos).y();
// }

void EuclideanClustering::GNSSCallback(const euclidean_clustering::Gnss::ConstPtr& gnss_msg) {
    gnss_pose = *gnss_msg;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> EuclideanClustering::euclidean_clustering() {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(statistical_outlier_removal_pcl, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // 탐색을 위한 KdTree 오브젝트 생성 //Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);  //KdTree 생성 

    // TODO: tunning
    double cluster_tolerance = 0.3;  // [m]
    int min_cluster_size = 15;
    int max_cluster_size = 100000;
    std::vector<pcl::PointIndices> cluster_indices;       // 군집화된 결과물의 Index 저장, 다중 군집화 객체는 cluster_indices[0] 순으로 저장 
    // 군집화 오브젝트 생성  
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(cloud);                    // 입력   
    ec.setClusterTolerance(cluster_tolerance);  // 2cm  
    ec.setMinClusterSize(min_cluster_size);     // 최소 포인트 수 
    ec.setMaxClusterSize(max_cluster_size);     // 최대 포인트 수
    ec.setSearchMethod(tree);                   // 위에서 정의한 탐색 방법 지정 
    ec.extract(cluster_indices);                // 군집화 적용 

    int cluster_num = cluster_indices.size();
    int intensity_num = 1;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    pcl::PointCloud<pcl::PointXYZI> TotalCloud;

    // std::cout << "cluster_indices: " << cluster_num << std::endl;

    for(auto itr = cluster_indices.begin(); itr != cluster_indices.end(); ++itr){
        pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        xyzi_cloud -> points.reserve(itr->indices.size());
        for(auto p_itr = itr -> indices.begin(); p_itr != itr -> indices.end(); ++p_itr){
            pcl::PointXYZ pt = cloud->points[*p_itr];
            pcl::PointXYZI pt2;
            pt2.x = pt.x;
            pt2.y = pt.y;
            pt2.z = pt.z;

            pt2.intensity = (float)(intensity_num+1);
            TotalCloud.push_back(pt2);
            xyzi_cloud -> points.push_back(pt2);
        }
        intensity_num++;
        clusters.push_back(xyzi_cloud);
    }

    std::cout << "cluster: " << clusters.size() << std::endl;

    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(TotalCloud, cloud_p);

    // sensor_msgs::PointCloud2 output; 
    pcl_conversions::fromPCL(cloud_p, euclidean_clustering_pcl);
    euclidean_clustering_pcl.header.frame_id = "laser_frame";

    return clusters;
}

void EuclideanClustering::CalcInfo(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& clusters) {
    int cluster_size = clusters->points.size();
    // std::cout << "cluster_size: " << cluster_size << std::endl;
    // std::cout << "===============================" << std::endl;
    double x_avg = 0;
    double y_avg = 0;
    double min_x = clusters -> points[0].x;
    double max_x = clusters -> points[0].x;
    double min_y = clusters -> points[0].y;
    double max_y = clusters -> points[0].y;

    for(auto& pt : clusters ->points){
        x_avg = x_avg + pt.x;
        y_avg = y_avg + pt.y;

        min_x = std::min(min_x,(double)pt.x);
        max_x = std::max(max_x,(double)pt.x);
        min_y = std::min(min_y,(double)pt.y);
        max_y = std::max(max_y,(double)pt.y);
    }

    x_avg = x_avg / cluster_size;
    y_avg = y_avg / cluster_size;
    x_avg = -x_avg;

    double size_x = abs(max_x - min_x);
    double size_y = abs(max_y - min_y);
    double size = std::max(size_x, size_y);
    double large_size = sqrt(pow(size_x, 2) + pow(size_y, 2));

    std::cout << "(x_avg, y_avg): " << x_avg << ", " << y_avg << " | size: " << size << std::endl;
    std::cout << "large_size: " << large_size << std::endl;

    std::pair<double, double> pair_latlong = CoordinateTranform(x_avg, y_avg);
    double object_lat = pair_latlong.first;
    double object_long = pair_latlong.second;
    std::cout << std::fixed << std::setprecision(7);
    std::cout << "(object_lat, object_long): " << object_lat << ", " << object_long << std::endl;
    std::cout << "-------------------" << std::endl;

    euclidean_clustering::ObjectInfo object_info;
    object_info.latitude = object_lat;
    object_info.longitude = object_long;
    object_info.size = size;
    // object_info.size = large_size;
    euclidean_clustering::ObjectInfoArray array;
    object_info_array.objectinfo.push_back(object_info);
    // std::cout << "object_info_array: " << object_info_array << std::endl;

    // double object_coordinate[3] = {object_lat, object_long, size};
    // // double object_coordinate[3] = {object_lat, object_long, large_size};
    // std::cout << "object_coordinate: " << object_coordinate[0] << ", " << object_coordinate[1] << ", " << object_coordinate[2] << std::endl;
    // object_array.data.push_back(object_coordinate[3]);
    // std::cout << "object_array: " << object_array << std::endl;
}

void EuclideanClustering::ObjectInfo(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cluster_clouds) {
    // object_array.data.clear();
    object_info_array.objectinfo.clear();
    for(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>::const_iterator itr = cluster_clouds.begin(); itr != cluster_clouds.end(); ++itr){
    CalcInfo(*itr);
    }
    std::cout << "object_info_array: " << object_info_array << std::endl;
}

std::pair<double, double> EuclideanClustering::CoordinateTranform(double input_x, double input_y) {
    // vehicle_x = 0.0;
    // vehicle_y = 0.0;
    // vehicle_yaw = M_PI/2;

    lanelet::projection::UtmProjector projection(lanelet::Origin({37.5422589, 127.0793964}));
    current_pos.lat = gnss_pose.latitude;
    current_pos.lon = gnss_pose.longitude;
    vehicle_x = projection.forward(current_pos).x();
    vehicle_y = projection.forward(current_pos).y();
    vehicle_yaw = gnss_pose.heading;

    double x = input_x - vehicle_x;
    double y = input_y - vehicle_y;
    double a = atan2(y, x) - vehicle_yaw;
    double d = sqrt( pow(x, 2) + pow(y, 2) );
    double output_x = d * cos(a);
    double output_y = d * sin(a);
    std::cout << "(output_x, output_y): " << output_x << ", " << output_y << std::endl;

    lanelet::BasicPoint3d utm_point(output_x, output_y, 0);
    lanelet::GPSPoint gps_point = projection.reverse(utm_point);
    double output_lat = gps_point.lat;
    double output_long = gps_point.lon;

    return std::make_pair(output_lat, output_long);
}

void EuclideanClustering::Print() {
    std::cout << "===============================" << std::endl;
}

void EuclideanClustering::Run() {
    if(statistical_outlier_removal_pcl.data.size() > 0) {
        cluster_clouds = euclidean_clustering();
        ObjectInfo(cluster_clouds);
        Print();
        Publish();
    }
}

void EuclideanClustering::Publish() {
    pub_pcl.publish(euclidean_clustering_pcl);
    pub_object_info_array.publish(object_info_array);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "euclidean_clustering");
    EuclideanClustering euclidean_clustering;
    ros::Rate loop_rate(30);    

    while(ros::ok()) {
        euclidean_clustering.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}