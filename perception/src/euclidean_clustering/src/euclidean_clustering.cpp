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

using namespace std;

class EuclideanClustering {
  private:
    ros::NodeHandle nh;
    ros::Publisher pub_pcl;
    ros::Subscriber sub_scan_pcl;
    sensor_msgs::PointCloud2 statistical_outlier_removal_pcl;
    sensor_msgs::PointCloud2 euclidean_clustering_pcl;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_clouds;

  public:
    EuclideanClustering();
    void sorCallback(const sensor_msgs::PointCloud2::ConstPtr& scan_pcl);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclidean_clustering();
    void calc_info(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& clusters);
    void object_info();
    void Run();
    void Publish();
};

EuclideanClustering::EuclideanClustering() {
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/euclidean_clustering_pcl2", 100);
    sub_scan_pcl = nh.subscribe("/statistical_outlier_removal_pcl2", 100, &EuclideanClustering::sorCallback, this);
}

void EuclideanClustering::sorCallback(const sensor_msgs::PointCloud2::ConstPtr& scan_pcl) {
    statistical_outlier_removal_pcl = *scan_pcl;
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
    double cluster_tolerance = 0.1;  // [m]
    int min_cluster_size = 5;
    int max_cluster_size = 1000;
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

void calc_info(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& clusters) {
    // clusters.
}

void object_info() {
    // for(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>::const_iterator itr = cluster_clouds.begin(); itr != cluster_clouds.end(); ++itr){
    // calc_info(*itr);
    // }
}

void EuclideanClustering::Run() {
    if(statistical_outlier_removal_pcl.data.size() > 0) {
        cluster_clouds = euclidean_clustering();
        // object_info();
        Publish();
    }
}

void EuclideanClustering::Publish() {
    pub_pcl.publish(euclidean_clustering_pcl);
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