#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <lanelet2_projection/UTM.h>
#include <euclidean_clustering/ObjectInfoArray.h>
#include <euclidean_clustering/ObjectInfo.h>
#include <detection_msgs/BoundingBoxes.h>
#include <fstream>
#include <sstream>
#include <string>
#include <queue>
#include <vector>
#include <algorithm>
#include <cmath>
#include <gps_imu_ekf/Gnss.h>
#include <unordered_set>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

struct Node;
vector<Node> init_nodes(const string& filename);
constexpr double R = 6371.0; // 지구 반지름 (km)

struct Node {
    int id;
    double x, y; // 위도, 경도 좌표
    double g, h, f; // g, h, f 값
    Node* parent; // 부모 노드
    vector<Node*> neighbors; // 간선에 해당, 포인터로 관리

    Node(int id = 0, double x = 0, double y = 0) // 생성자
        : id(id), x(x), y(y), g(0), h(0), f(0), parent(nullptr) {}
    
    bool operator==(const Node& other) const {
        return id == other.id; // 노드의 비교는 id로
    }
};

namespace std {
    template <>
    struct hash<Node> {
        std::size_t operator()(const Node& node) const {
            return std::hash<int>()(node.id); // id를 사용하여 해시 값을 계산
        }
    };
}

struct CompareNode { // 우선순위 큐 용
    bool operator()(Node* a, Node* b) const {
        return a->f > b->f;
    }
};

float heuristic(Node* start, Node* end) { // 휴리스틱 계산
    return sqrt(pow(start->x - end->x, 2) + pow(start->y - end->y, 2));
}

double getDistance(Node* start, Node* end) { // 노드간 거리 계산
    return sqrt(pow(start->x - end->x, 2) + pow(start->y - end->y, 2));
}

// 라디안으로 변환 함수
double toRadians(double degree) {
    return degree * M_PI / 180.0;
}

// 해버사인 공식을 사용하여 두 위도, 경도 간의 거리 계산 함수
double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dLat = toRadians(lat2 - lat1);
    double dLon = toRadians(lon2 - lon1);

    lat1 = toRadians(lat1);
    lat2 = toRadians(lat2);

    double a = sin(dLat / 2) * sin(dLat / 2) +
               sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c;
}

Node* find_current_node(vector<Node*>& global_path, double x, double y){
    double min = numeric_limits<double>::max();
    Node* result = nullptr;
    for(auto& node : global_path) {
        double distance = sqrt(pow(node->x - x, 2) + pow(node->y - y, 2));
        if(distance < min) {
            min = distance;
            result = node;
        }
    }
    return result;
}

Node* find_starting_node (vector<Node>& node_list, double x, double y){
    double min = numeric_limits<double>::max();
    Node* result = nullptr;
    for(auto& node : node_list) {
        double distance = sqrt(pow(node.x - x, 2) + pow(node.y - y, 2));
        if(distance < min) {
            min = distance;
            result = &node;
        }
    }
    return result;
}

Node* find_next_node(vector<Node*>& global_path, Node* current_node, int offset){
    int length = global_path.size();
    int current_node_index = -1;
    Node* result = nullptr;
    for(int i = 0; i < length - offset; i++){
        Node* tmp = global_path[i];
        if(tmp == current_node){
            current_node_index = i;
            break;
        }
    }
    if(current_node_index != -1){
        result = global_path[current_node_index + offset];
    }
    else{
        result = global_path[length - 1];
    }
    return result;
}

void draw_graph(vector<Node>& nodes) {
    for (auto& node : nodes) {
        std::string id_str = std::to_string(node.id);
        if (id_str.size() > 1) {
            std::string truncated_id_str = id_str.substr(1);
            int truncated_id = std::stoi(truncated_id_str);
            int target_id = truncated_id + 1;

            for (auto& other_node : nodes) {
                std::string other_id_str = std::to_string(other_node.id);
                if (other_id_str.size() > 1) {
                    int other_truncated_id = std::stoi(other_id_str.substr(1));
                    if (other_truncated_id == target_id) {
                        node.neighbors.push_back(&other_node);
                    }
                }
            }
        } else {
            std::cout << "Node ID: " << id_str << " (no first digit to remove)" << std::endl;
        }
    }
}

vector<Node*> Astar(Node* start, Node* end, vector<Node>& nodes, double obs_lat = -1, double obs_lon = -1, double obs_size = 0) {
    if (start == nullptr || end == nullptr) {
        cerr << "Error: Start or end node is null" << endl;
        return vector<Node*>();
    }
    priority_queue<Node*, vector<Node*>, CompareNode> openSet;
    unordered_set<Node*> set;
    unordered_set<Node*> closedSet;

    start->g = 0;
    start->h = heuristic(start, end);
    start->f = start->g + start->h;
    openSet.push(start);
    set.insert(start);
    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();
        set.erase(current);

        closedSet.insert(current);

        if (*current == *end) {
            vector<Node*> path;
            Node* current_2 = current;
            while (current_2 != start) {
                path.push_back(current_2);
                current_2 = current_2->parent;
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }
        
        for (Node* neighbor : current->neighbors) {
            if (closedSet.find(neighbor) != closedSet.end()) {
                continue;
            }

            if (obs_lat != -1 && obs_lon != -1 && obs_size != 0) {
                double obs_dist = haversine(neighbor->x, neighbor->y, obs_lat, obs_lon);
                if (obs_dist < obs_size) {
                    continue;
                }
            }

            double tentative_g_score = current->g + getDistance(current, neighbor);
            if (set.find(neighbor) == set.end()) {
                openSet.push(neighbor);
                set.insert(neighbor);
            } else if (tentative_g_score >= neighbor->g) {
                continue;
            }

            neighbor->parent = current;
            neighbor->g = tentative_g_score;
            neighbor->h = heuristic(neighbor, end);
            neighbor->f = neighbor->g + neighbor->h;
        }
    }

    return vector<Node*>();
}

vector<Node> init_nodes(const string& filename) {
    vector<Node> node_list;
    ifstream file(filename);
    string line;

    if (!file.is_open()) {
        cerr << "Could not open the file!" << endl;
        return node_list;
    }

    while (getline(file, line)) {
        stringstream ss(line);
        string node_declaration;

        if (getline(ss, node_declaration)) {
            string node_prefix = "Node ";
            if (node_declaration.find(node_prefix) == 0) {
                size_t start = node_declaration.find('(');
                size_t end = node_declaration.find(')');
                if (start != string::npos && end != string::npos) {
                    string params = node_declaration.substr(start + 1, end - start - 1);
                    stringstream param_ss(params);
                    string id_str, x_str, y_str;

                    if (getline(param_ss, id_str, ',') && getline(param_ss, x_str, ',') && getline(param_ss, y_str, ',')) {
                        int id = stoi(id_str);
                        double x = stod(x_str);
                        double y = stod(y_str);
                        node_list.emplace_back(id, x, y);
                    }
                }
            }
        }
    }

    file.close();
    return node_list;
}

class PathPlanner {
public:
    PathPlanner() {
        ros::NodeHandle nh;
        Astar_done = false;
        person_detected = false;
        car_detected = false;
        current_lat = 0.0;
        current_lon = 0.0;
        // string filename = "/home/test1/src/path_planner/src/node.csv";
         string filename = "/home/ain833437/test1/src/path_planner/src/node.csv";
       // string filename = "node.csv";
        nodes_ = init_nodes(filename);
    
        draw_graph(nodes_);
       
        sub_pose_ = nh.subscribe("kalman_pose", 1000, &PathPlanner::poseCallback, this);
        sub_obj_info_ = nh.subscribe("/object_info_array", 1000, &PathPlanner::objectInfoCallback, this);
        sub_bboxes_ = nh.subscribe("/yolov5/detections", 1000, &PathPlanner::boundingBoxCallback, this);
        pub_ = nh.advertise<std_msgs::Float32MultiArray>("next_node", 1000);
        pub_stop = nh.advertise<std_msgs::Bool>("stop_decision", 1000);
        map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map_view_out", 1);
    }

    void poseCallback(const gps_imu_ekf::Gnss::ConstPtr& msg) {
        current_lat = msg->latitude;
        current_lon = msg->longitude;

        if (person_detected) {
            ROS_INFO("Person detected, stopping the vehicle.");
            // return;
        }

        if (Astar_done == false) {
            Node* current_node = find_starting_node(nodes_, current_lat, current_lon);
            Node target_node = nodes_[nodes_.size() - 1];
            global_path_ = Astar(current_node, &target_node, nodes_);
            
            cout << "Astar done" << endl;
            Astar_done = true;
        }

        Node* current_node = find_current_node(global_path_, current_lat, current_lon);
        Node* next_node = find_next_node(global_path_, current_node, 5);

        std_msgs::Float32MultiArray next_node_msg;
        next_node_msg.data.resize(2);
        next_node_msg.data[0] = next_node->x;
        next_node_msg.data[1] = next_node->y;
        pub_.publish(next_node_msg);
        cout << "next_noe id:" << to_string(next_node->id)<<endl;
        cout << "next_node coord : " << to_string(next_node->x) << " " << to_string(next_node->y) << endl;
        
        //여기부터 플로팅
        lanelet::projection::UtmProjector projection(lanelet::Origin({37.5418003, 127.07848369999999}));
        pcl::PointCloud<pcl::PointXYZRGB> all_nodes;
        bool append = false;
        for(auto& node : nodes_){//모든 노드, 흰색 점
            pcl::PointXYZRGB point = pcl::PointXYZRGB(255,255,255); 
            current_pos.lat= node.x;
            current_pos.lon = node.y;
            point.x= projection.forward(current_pos).x();
            point.y= projection.forward(current_pos).y();
            point.z = 0;
            all_nodes.push_back(point);
        }
        for(auto& node : global_path_){//글로벌 경로 노드, 초록색
            if(node == current_node){append = true;}
            if(append == true){
            pcl::PointXYZRGB point = pcl::PointXYZRGB(0,255,0);
            current_pos.lat= node->x;
            current_pos.lon = node->y;
            point.x= projection.forward(current_pos).x();
            point.y= projection.forward(current_pos).y();
            point.z = 0;
            all_nodes.push_back(point);
            }
        }
        pcl::PointXYZRGB point = pcl::PointXYZRGB(255,0,0);//현재 차량의 위치, 빨간색
        current_pos.lat= current_node->x;
        current_pos.lon = current_node->y;
        point.x= projection.forward(current_pos).x();
        point.y= projection.forward(current_pos).y();
        point.z = 0;
        all_nodes.push_back(point);

        point = pcl::PointXYZRGB(0,0,255);//next_node 위치, 파란색
        current_pos.lat= next_node->x;
        current_pos.lon = next_node->y;
        point.x= projection.forward(current_pos).x();
        point.y= projection.forward(current_pos).y();
        point.z = 0;
        all_nodes.push_back(point);

        sensor_msgs::PointCloud2 map_view;
        pcl::toROSMsg(all_nodes, map_view);
        map_view.header = msg->header;
        map_view.header.frame_id ="map";
        map_pub_.publish(map_view);
    }

    void objectInfoCallback(const euclidean_clustering::ObjectInfoArray::ConstPtr& msg) {
        if (person_detected) {
            ROS_INFO("Person detected, stopping the vehicle.");
            return;
        }

        for (const auto& obj : msg->objectinfo) {
            double obs_lat = obj.latitude;
            double obs_lon = obj.longitude;
            double obs_size = obj.size;

            if (car_detected) {
                Node* current_node = find_starting_node(nodes_, obs_lat, obs_lon);
                Node target_node = nodes_[nodes_.size() - 1];
                global_path_ = Astar(current_node, &target_node, nodes_, obs_lat, obs_lon, obs_size);
                cout << "Replanning path to avoid obstacle" << endl;
            }
        }
    }

    void boundingBoxCallback(const detection_msgs::BoundingBoxes::ConstPtr& msg) {
        
        person_detected_temp = false;
        car_detected_temp = false;

        for (const auto& bbox : msg->bounding_boxes) {
            if (bbox.Class == "pedestrian") {
                //double person_lat = bbox.ymin;
                //double person_lon = bbox.xmin;
                //double distance_to_person = haversine(current_lat, current_lon, person_lat, person_lon);

               // if (distance_to_person <= 5.0) {
                  person_detected_temp = false;
             //   }
            } else if (bbox.Class == "car") {
                car_detected_temp = false;
            }
        }

        person_detected = person_detected_temp;
        car_detected = car_detected_temp;
        std_msgs::Bool stop_sign;
        stop_sign.data = person_detected;
        pub_stop.publish(stop_sign);
    }
    void reset(){
        person_detected = false;
        car_detected = false;
        std_msgs::Bool stop_sign;
        stop_sign.data = person_detected;
        pub_stop.publish(stop_sign);
    }

private:
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_obj_info_;
    ros::Subscriber sub_bboxes_;
    ros::Publisher pub_;
    ros::Publisher pub_stop;
    vector<Node> nodes_;
    vector<Node*> global_path_;
    ros::Publisher map_pub_;
    bool Astar_done;
    bool person_detected;
    bool car_detected;
    double current_lat;
    double current_lon;
    bool person_detected_temp;
    bool car_detected_temp;
    lanelet::GPSPoint current_pos;
};  


int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_node");
    
    PathPlanner path_planner;

    ros::Rate loop_rate(4); // 0.25초마다 실행

    while (ros::ok()) {
        path_planner.reset();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
