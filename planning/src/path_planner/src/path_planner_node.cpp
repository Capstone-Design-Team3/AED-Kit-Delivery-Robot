#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/NavSatFix.h> // GPS 데이터를 위한 메시지
#include <fstream>
#include <sstream>
#include <string>
#include <queue>
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_set>


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
    // 유클리드 거리 사용
    return sqrt(pow(start->x - end->x, 2) + pow(start->y - end->y, 2));
}

double getDistance(Node* start, Node* end) { // 노드간 거리 계산
    return sqrt(pow(start->x - end->x, 2) + pow(start->y - end->y, 2));
}

Node* find_current_node(vector<Node*>& global_path, double x, double y){ //현재 차량의 좌표, localization 팀으로 부터 전달받은 lat, lon  좌표
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
Node* find_starting_node (vector<Node>& node_list, double x, double y){ //현재 차량의 좌표, localization 팀으로 부터 전달받은 lat, lon  좌표
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

Node* find_next_node(vector<Node*>& global_path, Node* current_node, int offset){ //글로벌 패스와 커런트 노드, 얼마나 앞에 있는 노드를 반환할것 인지 offset 설정
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
            std::string truncated_id_str = id_str.substr(1); // 첫 번째 문자를 무시
            int truncated_id = std::stoi(truncated_id_str);
            int target_id = truncated_id + 1;
            //std::cout << "Node ID without first digit: " << truncated_id << std::endl;
            //std::cout << "Nodes with ID " << target_id << " when ignoring the first digit:" << std::endl;

            // 원하는 트렁크 ID를 가진 노드를 찾고 출력
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

vector<Node*> Astar(Node* start, Node* end) { // A* 알고리즘
    if (start == nullptr || end == nullptr) {
        cerr << "Error: Start or end node is null" << endl;
        return vector<Node*>();
    }
    int count = 0;
    priority_queue<Node*, vector<Node*>, CompareNode> openSet; // 오픈리스트
    unordered_set<Node*> set; // 오픈 리스트에 특정 객체 존재 여부 확인용
    unordered_set<Node*> closedSet; // 닫힌리스트

    start->g = 0; // 시작점이니까 0
    start->h = heuristic(start, end); // h 계산
    start->f = start->g + start->h; // f = g + h
    openSet.push(start); // 오픈리스트(우선순위 큐)에 추가
    set.insert(start); // 나중에 오픈리스트에 값이 존재하는지 알아야 해서
    while (!openSet.empty()) { // 오픈리스트가 빌 때까지 반복
        Node* current = openSet.top(); // f 값이 가장 작은 노드
        openSet.pop(); // f 값이 가장 작은 노드를 제거
        set.erase(current); // 여기서도 제거

        closedSet.insert(current); // 닫힌 리스트에 현재 노드 추가

        if (*current == *end) { // 목적지에 도달했으면 경로를 반환
        
            vector<Node*> path; // 결과 경로 리스트
            Node* current_2 = current;
            while (current_2 != start) { // 시작점까지 부모를 따라가면서 경로를 저장
                path.push_back(current_2);
                current_2 = current_2->parent;
            }
            path.push_back(start); // 시작점도 추가
            reverse(path.begin(), path.end()); // 시작부터 끝까지 순서를 바꿈
            return path; // 최단 경로 반환
        }
        
        for (Node* neighbor : current->neighbors) { // 현재 노드의 이웃에 대해 반복
            if (closedSet.find(neighbor) != closedSet.end()) { // 이웃이 닫힌 리스트에 있으면 건너뜀
                continue;
            }

            double tentative_g_score = current->g + getDistance(current, neighbor); // 새로운 g 값 계산
            if (set.find(neighbor) == set.end()) { // 이웃이 오픈 리스트에 포함되어 있지 않으면 추가
                openSet.push(neighbor);
                set.insert(neighbor);
            } else if (tentative_g_score >= neighbor->g) { // 이미 이웃이 오픈 리스트에 있고, 더 나쁜 경로인 경우 스킵
                continue;
            }

            // 새로운 경로로 이웃의 정보 업데이트
            neighbor->parent = current;
            neighbor->g = tentative_g_score;
            neighbor->h = heuristic(neighbor, end);
            neighbor->f = neighbor->g + neighbor->h;
        }
    }

    return vector<Node*>(); // 경로를 찾지 못한 경우 빈 리스트 반환
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

        // CSV 파일의 각 라인은 'Node nodeID(id, x, y);' 형식이어야 합니다.
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

// ROS 노드 클래스
class PathPlanner {
public:
    PathPlanner() {
        // 노드 초기화
        ros::NodeHandle nh;
        Astar_done = false;
        // Subscriber와 Publisher 설정
        // 노드 초기화
        string filename = "/home/hyeongju/graduate/catkin_ws/src/path_planner/src/node.csv";
        nodes_ = init_nodes(filename);
    
        draw_graph(nodes_);
       
        sub_ = nh.subscribe("current_pose", 1000, &PathPlanner::poseCallback, this);
        pub_ = nh.advertise<std_msgs::Float32MultiArray>("next_node", 1000);
    }
   
    void poseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        double current_lat = msg->latitude;
        double current_lon = msg->longitude;

        if(Astar_done == false){
            Node* current_node = find_starting_node(nodes_,current_lat, current_lon);
            Node target_node = nodes_[nodes_.size()-1];
            global_path_ = Astar(current_node, &target_node); // 예시
            
            cout << "Astar done" << endl;
            Astar_done = true;
        }
        Node* current_node = find_current_node(global_path_, current_lat, current_lon);
        Node* next_node = find_next_node(global_path_, current_node, 3); // 예시

        
        std_msgs::Float32MultiArray next_node_msg;
        next_node_msg.data.resize(2);
        next_node_msg.data[0] = next_node->x;
        next_node_msg.data[1] = next_node->y;
        pub_.publish(next_node_msg);
        cout << "next_node coord : "<<to_string(next_node->x) +" "+ to_string(next_node->y) <<endl;
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    vector<Node> nodes_;
    vector<Node*> global_path_;
    bool Astar_done;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_node");
    
    PathPlanner path_planner;

    ros::Rate loop_rate(2); // 0.5초마다 실행

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
