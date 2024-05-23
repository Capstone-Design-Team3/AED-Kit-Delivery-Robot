#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <queue>
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_set>

using namespace std;

constexpr double R = 6371.0; // 지구 반지름 (km)

struct Node {
    int id;
    double x, y;//let, lon 좌표를 입력
    double g, h, f;//g,f는 처음에 그냥 0으로 초기화, h는 휴리스틱 함수로 초기화
    Node* parent; //null로 초기화
    vector<Node> neighbors; //***간선에 해당, 직접 만들어야함***

    Node(int id = 0, double x = 0, double y=0) //이거로만 생성
        : id(id), x(x), y(y), g(0), h(0), f(0), parent(nullptr) {}
    
};

struct CompareNode {//우선순위 큐 용
    bool operator()(Node* const& a, Node* const& b) const {
        return a->f > b->f;
    }
};

float heuristic(Node start, Node end) {//이거로 휴리스틱 계산해서 neighbors 만들기
    //휴리스틱으로 유클리드 거리 사용
    return sqrt(pow(start.x - end.x, 2) + pow(start.y - end.y, 2));
}

float getDistance(Node start, Node end){//사실 위와 똑같은 코드, 노드간 거리 계산, 구분하기 위해 이름만 바꿈
    return sqrt(pow(start.x - end.x, 2) + pow(start.y - end.y, 2));
}

// 경도 위도로 거리를 구하기 위한 함수
double getDistance_flat(Node start, Node end) {

    double dLat = (start.x - end.x) * M_PI / 180.0;
    double dLon = (start.y - end.y) * M_PI / 180.0;
    double a = std::sin(dLat/2) * std::sin(dLat/2) +
               std::cos(start.x * M_PI / 180.0) * std::cos(end.x* M_PI / 180.0) *
               std::sin(dLon/2) * std::sin(dLon/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    double distance = R * c;
    return distance;
}

double heuristic_flat(Node start, Node end) {//이거로 휴리스틱 계산해서 neighbors 만들기

    double dLat = (start.x - end.x) * M_PI / 180.0;
    double dLon = (start.y - end.y) * M_PI / 180.0;
    double a = std::sin(dLat/2) * std::sin(dLat/2) +
               std::cos(start.x * M_PI / 180.0) * std::cos(end.x* M_PI / 180.0) *
               std::sin(dLon/2) * std::sin(dLon/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    double distance = R * c;
    return distance;
}
void draw_graph(vector<Node> nodes) {
    for (const auto node : nodes) {
        std::string id_str = std::to_string(node.id);
        if (id_str.size() > 1) {
            std::string truncated_id_str = id_str.substr(1); // Ignore the first character
            int truncated_id = std::stoi(truncated_id_str);
            int target_id = truncated_id + 1;
            std::cout << "Node ID without first digit: " << truncated_id << std::endl;
            std::cout << "Nodes with ID " << target_id << " when ignoring the first digit:" << std::endl;

            // Find and print nodes with the desired truncated ID
            for (const auto other_node : nodes) {
                std::string other_id_str = std::to_string(other_node.id);
                if (other_id_str.size() > 1) {
                    int other_truncated_id = std::stoi(other_id_str.substr(1));
                    if (other_truncated_id == target_id) {
                        node.neighbors.push_back(other_node);
                    }
                }
            }
        } else {
            std::cout << "Node ID: " << id_str << " (no first digit to remove)" << std::endl;
        }
    }
}


void init_nodes(vector<Node> node_list){//python 파일 돌려서 나온 터미널 복붙
Node node26554(26554,37.54231962228,127.07856361427);
node_list.push_back(node26554);
Node node26555(26555,37.54231693787,127.07855607638);
node_list.push_back(node26555);
Node node26556(26556,37.54231009752,127.07855279417);
node_list.push_back(node26556);
Node node26557(26557,37.54230259258,127.0785516589);
node_list.push_back(node26557);
Node node26558(26558,37.54229432426,127.07855162945);
node_list.push_back(node26558);
Node node26559(26559,37.54228472673,127.07855156472);
node_list.push_back(node26559);
Node node26560(26560,37.54227479505,127.07855076472);
node_list.push_back(node26560);
Node node26561(26561,37.5422633,127.0785503);
node_list.push_back(node26561);
Node node26562(26562,37.5422536,127.0785487);
node_list.push_back(node26562);
Node node26563(26563,37.5422438,127.0785474);
node_list.push_back(node26563);
Node node26564(26564,37.542234,127.0785459);
node_list.push_back(node26564);
Node node26565(26565,37.5422247,127.0785447);
node_list.push_back(node26565);
Node node26566(26566,37.5422158,127.0785433);
node_list.push_back(node26566);
Node node26567(26567,37.542207,127.0785415);
node_list.push_back(node26567);
Node node26568(26568,37.5421977,127.0785406);
node_list.push_back(node26568);
Node node26569(26569,37.5421885,127.0785391);
node_list.push_back(node26569);
Node node26570(26570,37.5421793,127.0785377);
node_list.push_back(node26570);
Node node26571(26571,37.5421699,127.0785362);
node_list.push_back(node26571);
Node node26572(26572,37.5421605,127.0785349);
node_list.push_back(node26572);
Node node26573(26573,37.5421506,127.0785334);
node_list.push_back(node26573);
Node node26574(26574,37.5421411,127.0785318);
node_list.push_back(node26574);
Node node26575(26575,37.5421311,127.0785308);
node_list.push_back(node26575);
Node node26576(26576,37.5421215,127.0785298);
node_list.push_back(node26576);
Node node26577(26577,37.5421118,127.0785284);
node_list.push_back(node26577);
Node node26578(26578,37.5421024,127.0785269);
node_list.push_back(node26578);
Node node26579(26579,37.5420926,127.0785258);
node_list.push_back(node26579);
Node node26580(26580,37.542083,127.0785249);
node_list.push_back(node26580);
Node node26581(26581,37.5420731,127.0785239);
node_list.push_back(node26581);
Node node26582(26582,37.542063,127.0785227);
node_list.push_back(node26582);
Node node26583(26583,37.5420533,127.0785204);
node_list.push_back(node26583);
Node node26584(26584,37.5420443,127.0785185);
node_list.push_back(node26584);
Node node26585(26585,37.5420342,127.0785169);
node_list.push_back(node26585);
Node node26586(26586,37.5420248,127.0785153);
node_list.push_back(node26586);
Node node26587(26587,37.5420155,127.0785138);
node_list.push_back(node26587);
Node node26588(26588,37.5420059,127.0785127);
node_list.push_back(node26588);
Node node26589(26589,37.5419963,127.0785107);
node_list.push_back(node26589);
Node node26590(26590,37.5419866,127.0785095);
node_list.push_back(node26590);
Node node26591(26591,37.5419773,127.078508);
node_list.push_back(node26591);
Node node26592(26592,37.5419677,127.0785073);
node_list.push_back(node26592);
Node node26593(26593,37.5419579,127.0785063);
node_list.push_back(node26593);
Node node26594(26594,37.5419481,127.0785054);
node_list.push_back(node26594);
Node node26595(26595,37.5419378,127.0785044);
node_list.push_back(node26595);
Node node26596(26596,37.5419274,127.0785028);
node_list.push_back(node26596);
Node node26597(26597,37.5419174,127.078501);
node_list.push_back(node26597);
Node node26598(26598,37.5419077,127.0784991);
node_list.push_back(node26598);
Node node26599(26599,37.5418985,127.0784977);
node_list.push_back(node26599);
Node node26600(26600,37.5418884,127.078496);
node_list.push_back(node26600);
Node node26601(26601,37.5418784,127.0784947);
node_list.push_back(node26601);
Node node26602(26602,37.5418688,127.0784934);
node_list.push_back(node26602);
Node node26603(26603,37.5418587,127.078492);
node_list.push_back(node26603);
Node node26604(26604,37.5418481,127.0784905);
node_list.push_back(node26604);
Node node26605(26605,37.5418377,127.0784892);
node_list.push_back(node26605);
Node node26606(26606,37.5418278,127.0784879);
node_list.push_back(node26606);
Node node26607(26607,37.5418181,127.0784867);
node_list.push_back(node26607);
Node node26608(26608,37.5418082,127.0784856);
node_list.push_back(node26608);
Node node26609(26609,37.5417979,127.0784841);
node_list.push_back(node26609);
Node node26610(26610,37.5417877,127.0784825);
node_list.push_back(node26610);
Node node26611(26611,37.5417773,127.0784808);
node_list.push_back(node26611);
Node node26612(26612,37.5417668,127.0784791);
node_list.push_back(node26612);
Node node26613(26613,37.5417562,127.0784776);
node_list.push_back(node26613);
Node node26614(26614,37.5417464,127.0784761);
node_list.push_back(node26614);
Node node26615(26615,37.5417366,127.0784743);
node_list.push_back(node26615);
Node node26616(26616,37.5417269,127.0784736);
node_list.push_back(node26616);
Node node26617(26617,37.541717,127.0784721);
node_list.push_back(node26617);
Node node26619(26619,37.5417063,127.0784707);
node_list.push_back(node26619);
Node node26622(26622,37.54202576847,127.07850070453);
node_list.push_back(node26622);
Node node26623(26623,37.54192836847,127.07848820453);
node_list.push_back(node26623);
Node node26624(26624,37.54226426846,127.07853570453);
node_list.push_back(node26624);
Node node26625(26625,37.54172786848,127.07845900453);
node_list.push_back(node26625);
Node node26626(26626,37.54217086846,127.07852160453);
node_list.push_back(node26626);
Node node26627(26627,37.54222566846,127.07853010453);
node_list.push_back(node26627);
Node node26628(26628,37.54197826847,127.07849340453);
node_list.push_back(node26628);
Node node26629(26629,37.54203516847,127.07850230453);
node_list.push_back(node26629);
Node node26630(26630,37.54193876847,127.07848980453);
node_list.push_back(node26630);
Node node26632(26632,37.54227416846,127.07853650453);
node_list.push_back(node26632);
Node node26633(26633,37.54173756848,127.07845970453);
node_list.push_back(node26633);
Node node26634(26634,37.54232537589,127.07854851588);
node_list.push_back(node26634);
Node node26635(26635,37.54223496846,127.07853130453);
node_list.push_back(node26635);
Node node26636(26636,37.54198756847,127.07849490453);
node_list.push_back(node26636);
Node node26637(26637,37.54204526847,127.07850390453);
node_list.push_back(node26637);
Node node26638(26638,37.54178866847,127.07846790453);
node_list.push_back(node26638);
Node node26640(26640,37.54228356846,127.07853730453);
node_list.push_back(node26640);
Node node26641(26641,37.54174736847,127.07846150453);
node_list.push_back(node26641);
Node node26642(26642,37.54224476846,127.07853280453);
node_list.push_back(node26642);
Node node26643(26643,37.54199726847,127.07849610453);
node_list.push_back(node26643);
Node node26644(26644,37.54205426847,127.07850580453);
node_list.push_back(node26644);
Node node26645(26645,37.54179886847,127.07846950453);
node_list.push_back(node26645);
Node node26646(26646,37.54170726847,127.07845610453);
node_list.push_back(node26646);
Node node26647(26647,37.54229236846,127.07853770453);
node_list.push_back(node26647);
Node node26648(26648,37.54175716847,127.07846300453);
node_list.push_back(node26648);
Node node26649(26649,37.54210336847,127.07851230453);
node_list.push_back(node26649);
Node node26650(26650,37.54200686847,127.07849810453);
node_list.push_back(node26650);
Node node26651(26651,37.54206396847,127.07850810453);
node_list.push_back(node26651);
Node node26652(26652,37.54180916847,127.07847100453);
node_list.push_back(node26652);
Node node26653(26653,37.54186976847,127.07847880453);
node_list.push_back(node26653);
Node node26654(26654,37.54230116846,127.07853840453);
node_list.push_back(node26654);
Node node26655(26655,37.54176776847,127.07846450453);
node_list.push_back(node26655);
Node node26656(26656,37.54211276846,127.07851380453);
node_list.push_back(node26656);
Node node26657(26657,37.54201646847,127.07849920453);
node_list.push_back(node26657);
Node node26658(26658,37.54207406847,127.07850930453);
node_list.push_back(node26658);
Node node26659(26659,37.54181906847,127.07847210453);
node_list.push_back(node26659);
Node node26660(26660,37.54187936847,127.07848010453);
node_list.push_back(node26660);
Node node26661(26661,37.54231026846,127.07853920453);
node_list.push_back(node26661);
Node node26662(26662,37.54177826847,127.07846620453);
node_list.push_back(node26662);
Node node26663(26663,37.54212246846,127.07851520453);
node_list.push_back(node26663);
Node node26664(26664,37.54218026846,127.07852310453);
node_list.push_back(node26664);
Node node26665(26665,37.54208396847,127.07851030453);
node_list.push_back(node26665);
Node node26666(26666,37.54182876847,127.07847330453);
node_list.push_back(node26666);
Node node26667(26667,37.54231936846,127.07854215146);
node_list.push_back(node26667);
Node node26668(26668,37.54188936847,127.07848140453);
node_list.push_back(node26668);
Node node26669(26669,37.54213206846,127.07851620453);
node_list.push_back(node26669);
Node node26670(26670,37.54218946846,127.07852450453);
node_list.push_back(node26670);
Node node26671(26671,37.54209356847,127.07851120453);
node_list.push_back(node26671);
Node node26672(26672,37.54183866847,127.07847460453);
node_list.push_back(node26672);
Node node26673(26673,37.54189946847,127.07848310453);
node_list.push_back(node26673);
Node node26674(26674,37.54214206846,127.07851720453);
node_list.push_back(node26674);
Node node26675(26675,37.54219866846,127.07852600453);
node_list.push_back(node26675);
Node node26676(26676,37.54194906847,127.07849080453);
node_list.push_back(node26676);
Node node26677(26677,37.54184906847,127.07847590453);
node_list.push_back(node26677);
Node node26678(26678,37.54190866847,127.07848450453);
node_list.push_back(node26678);
Node node26680(26680,37.54215156846,127.07851880453);
node_list.push_back(node26680);
Node node26681(26681,37.54220796846,127.07852690453);
node_list.push_back(node26681);
Node node26682(26682,37.54195886847,127.07849170453);
node_list.push_back(node26682);
Node node26683(26683,37.54185966847,127.07847740453);
node_list.push_back(node26683);
Node node26684(26684,37.54191836847,127.07848640453);
node_list.push_back(node26684);
Node node26685(26685,37.54225456846,127.07853410453);
node_list.push_back(node26685);
Node node26686(26686,37.54171796848,127.07845750453);
node_list.push_back(node26686);
Node node26687(26687,37.54216146846,127.07852030453);
node_list.push_back(node26687);
Node node26688(26688,37.54221676846,127.07852870453);
node_list.push_back(node26688);
Node node26689(26689,37.54196866847,127.07849270453);
node_list.push_back(node26689);
Node node26702(26702,37.54232112189,127.07857399753);
node_list.push_back(node26702);
Node node26758(26758,37.5422647,127.0793916);
node_list.push_back(node26758);
Node node26759(26759,37.5422648,127.0793828);
node_list.push_back(node26759);
Node node26760(26760,37.5422651,127.0793728);
node_list.push_back(node26760);
Node node26761(26761,37.5422654,127.0793635);
node_list.push_back(node26761);
Node node26762(26762,37.5422664,127.0793539);
node_list.push_back(node26762);
Node node26763(26763,37.542267,127.0793437);
node_list.push_back(node26763);
Node node26764(26764,37.5422676,127.0793339);
node_list.push_back(node26764);
Node node26765(26765,37.5422682,127.0793236);
node_list.push_back(node26765);
Node node26766(26766,37.5422692,127.0793126);
node_list.push_back(node26766);
Node node26767(26767,37.5422697,127.0793016);
node_list.push_back(node26767);
Node node26768(26768,37.5422706,127.0792904);
node_list.push_back(node26768);
Node node26769(26769,37.5422723,127.0792793);
node_list.push_back(node26769);
Node node26770(26770,37.5422742,127.0792686);
node_list.push_back(node26770);
Node node26771(26771,37.5422755,127.0792575);
node_list.push_back(node26771);
Node node26772(26772,37.5422767,127.079246);
node_list.push_back(node26772);
Node node26773(26773,37.5422777,127.0792345);
node_list.push_back(node26773);
Node node26774(26774,37.5422783,127.0792232);
node_list.push_back(node26774);
Node node26775(26775,37.5422791,127.0792118);
node_list.push_back(node26775);
Node node26776(26776,37.54228,127.0792003);
node_list.push_back(node26776);
Node node26777(26777,37.5422809,127.0791888);
node_list.push_back(node26777);
Node node26778(26778,37.5422818,127.0791781);
node_list.push_back(node26778);
Node node26779(26779,37.5422829,127.0791674);
node_list.push_back(node26779);
Node node26780(26780,37.5422839,127.079156);
node_list.push_back(node26780);
Node node26781(26781,37.5422852,127.0791446);
node_list.push_back(node26781);
Node node26782(26782,37.5422865,127.0791334);
node_list.push_back(node26782);
Node node26783(26783,37.5422874,127.0791227);
node_list.push_back(node26783);
Node node26784(26784,37.5422884,127.0791115);
node_list.push_back(node26784);
Node node26785(26785,37.5422892,127.0790999);
node_list.push_back(node26785);
Node node26786(26786,37.5422902,127.0790884);
node_list.push_back(node26786);
Node node26787(26787,37.5422915,127.0790766);
node_list.push_back(node26787);
Node node26788(26788,37.5422924,127.0790652);
node_list.push_back(node26788);
Node node26789(26789,37.5422935,127.0790533);
node_list.push_back(node26789);
Node node26790(26790,37.5422945,127.0790414);
node_list.push_back(node26790);
Node node26791(26791,37.5422957,127.0790297);
node_list.push_back(node26791);
Node node26792(26792,37.5422968,127.0790179);
node_list.push_back(node26792);
Node node26793(26793,37.5422979,127.0790059);
node_list.push_back(node26793);
Node node26794(26794,37.5422988,127.0789943);
node_list.push_back(node26794);
Node node26795(26795,37.5422997,127.078983);
node_list.push_back(node26795);
Node node26796(26796,37.5423005,127.0789712);
node_list.push_back(node26796);
Node node26797(26797,37.5423011,127.0789595);
node_list.push_back(node26797);
Node node26798(26798,37.5423019,127.0789477);
node_list.push_back(node26798);
Node node26799(26799,37.542303,127.0789359);
node_list.push_back(node26799);
Node node26800(26800,37.542304,127.078924);
node_list.push_back(node26800);
Node node26801(26801,37.542305,127.0789127);
node_list.push_back(node26801);
Node node26802(26802,37.5423057,127.0789009);
node_list.push_back(node26802);
Node node26803(26803,37.5423065,127.0788892);
node_list.push_back(node26803);
Node node26804(26804,37.5423073,127.0788779);
node_list.push_back(node26804);
Node node26805(26805,37.5423082,127.0788665);
node_list.push_back(node26805);
Node node26806(26806,37.5423091,127.0788547);
node_list.push_back(node26806);
Node node26807(26807,37.5423098,127.0788426);
node_list.push_back(node26807);
Node node26808(26808,37.5423102,127.0788311);
node_list.push_back(node26808);
Node node26809(26809,37.5423106,127.078819);
node_list.push_back(node26809);
Node node26810(26810,37.5423111,127.078807);
node_list.push_back(node26810);
Node node26811(26811,37.5423122,127.0787961);
node_list.push_back(node26811);
Node node26812(26812,37.542313,127.0787852);
node_list.push_back(node26812);
Node node26813(26813,37.542314,127.0787737);
node_list.push_back(node26813);
Node node26814(26814,37.5423153,127.0787614);
node_list.push_back(node26814);
Node node26815(26815,37.5423162,127.0787492);
node_list.push_back(node26815);
Node node26816(26816,37.5423168,127.0787373);
node_list.push_back(node26816);
Node node26817(26817,37.5423174,127.0787255);
node_list.push_back(node26817);
Node node26818(26818,37.5423182,127.0787132);
node_list.push_back(node26818);
Node node26819(26819,37.5423191,127.0787007);
node_list.push_back(node26819);
Node node26820(26820,37.5423199,127.0786884);
node_list.push_back(node26820);
Node node26821(26821,37.5423205,127.0786763);
node_list.push_back(node26821);
Node node26822(26822,37.5423211,127.0786647);
node_list.push_back(node26822);
Node node26823(26823,37.5423221,127.0786532);
node_list.push_back(node26823);
Node node26824(26824,37.5423231,127.0786414);
node_list.push_back(node26824);
Node node26825(26825,37.542324,127.0786293);
node_list.push_back(node26825);
Node node26826(26826,37.5423251,127.0786168);
node_list.push_back(node26826);
Node node26827(26827,37.5423263,127.0786045);
node_list.push_back(node26827);
Node node26828(26828,37.5423278,127.0785925);
node_list.push_back(node26828);
Node node26829(26829,37.5423286,127.07858308221);
node_list.push_back(node26829);
Node node26830(26830,37.54232895458,127.07857418804);
node_list.push_back(node26830);
Node node26831(26831,37.54232930173,127.0785663615);
node_list.push_back(node26831);
Node node26832(26832,37.54232808119,127.07855781166);
node_list.push_back(node26832);
Node node26918(26918,37.54226147267,127.07932308166);
node_list.push_back(node26918);
Node node26919(26919,37.54231317268,127.07868788166);
node_list.push_back(node26919);
Node node26920(26920,37.54226557267,127.07927878166);
node_list.push_back(node26920);
Node node26921(26921,37.54228677268,127.07905278166);
node_list.push_back(node26921);
Node node26922(26922,37.54231727268,127.07862878166);
node_list.push_back(node26922);
Node node26923(26923,37.54225797267,127.07939108166);
node_list.push_back(node26923);
Node node26924(26924,37.54229207268,127.07899378166);
node_list.push_back(node26924);
Node node26925(26925,37.54230857268,127.07876088166);
node_list.push_back(node26925);
Node node26926(26926,37.54226027267,127.07934318166);
node_list.push_back(node26926);
Node node26927(26927,37.54231147268,127.07871268166);
node_list.push_back(node26927);
Node node26928(26928,37.54228067268,127.07912218166);
node_list.push_back(node26928);
Node node26929(26929,37.54228477268,127.07907608166);
node_list.push_back(node26929);
Node node26930(26930,37.54230307268,127.07884208166);
node_list.push_back(node26930);
Node node26931(26931,37.54227327268,127.07919978166);
node_list.push_back(node26931);
Node node26932(26932,37.54230627268,127.07878468166);
node_list.push_back(node26932);
Node node26933(26933,37.54225867267,127.07936298166);
node_list.push_back(node26933);
Node node26934(26934,37.54227847268,127.07914408166);
node_list.push_back(node26934);
Node node26935(26935,37.54228247268,127.07909938166);
node_list.push_back(node26935);
Node node26936(26936,37.54230147268,127.07886598166);
node_list.push_back(node26936);
Node node26937(26937,37.54227157267,127.07922268166);
node_list.push_back(node26937);
Node node26938(26938,37.54230437268,127.07880648166);
node_list.push_back(node26938);
Node node26939(26939,37.54227617268,127.07916688166);
node_list.push_back(node26939);
Node node26940(26940,37.54229627268,127.07893538166);
node_list.push_back(node26940);
Node node26941(26941,37.54229977268,127.07888868166);
node_list.push_back(node26941);
Node node26942(26942,37.54226387267,127.07928988166);
node_list.push_back(node26942);
Node node26943(26943,37.54231537268,127.07865268166);
node_list.push_back(node26943);
Node node26944(26944,37.54226997267,127.07924548166);
node_list.push_back(node26944);
Node node26945(26945,37.54229007268,127.07901738166);
node_list.push_back(node26945);
Node node26946(26946,37.54232107268,127.07859198166);
node_list.push_back(node26946);
Node node26947(26947,37.54227417268,127.07918828166);
node_list.push_back(node26947);
Node node26948(26948,37.54229437268,127.07895898166);
node_list.push_back(node26948);
Node node26949(26949,37.54229827268,127.07891218166);
node_list.push_back(node26949);
Node node26950(26950,37.54226247267,127.07931208166);
node_list.push_back(node26950);
Node node26951(26951,37.54231377268,127.07867578166);
node_list.push_back(node26951);
Node node26952(26952,37.54226747267,127.07926808166);
node_list.push_back(node26952);
Node node26953(26953,37.54228777268,127.07904088166);
node_list.push_back(node26953);
Node node26954(26954,37.54231837268,127.07861628166);
node_list.push_back(node26954);
Node node26955(26955,37.54225807267,127.07938228166);
node_list.push_back(node26955);
Node node26956(26956,37.54229297268,127.07898248166);
node_list.push_back(node26956);
Node node26957(26957,37.54230947268,127.07874868166);
node_list.push_back(node26957);
Node node26958(26958,37.54226087267,127.07933338166);
node_list.push_back(node26958);
Node node26959(26959,37.54231237268,127.07870018166);
node_list.push_back(node26959);
Node node26960(26960,37.54228167268,127.07911098166);
node_list.push_back(node26960);
Node node26961(26961,37.54228567268,127.07906468166);
node_list.push_back(node26961);
Node node26962(26962,37.54230347268,127.07883058166);
node_list.push_back(node26962);
Node node26963(26963,37.54229117268,127.07900538166);
node_list.push_back(node26963);
Node node26964(26964,37.54230727268,127.07877318166);
node_list.push_back(node26964);
Node node26965(26965,37.54225967267,127.07935338166);
node_list.push_back(node26965);
Node node26966(26966,37.54231067268,127.07872498166);
node_list.push_back(node26966);
Node node26967(26967,37.54227977268,127.07913288166);
node_list.push_back(node26967);
Node node26968(26968,37.54228347268,127.07908788166);
node_list.push_back(node26968);
Node node26969(26969,37.54230237268,127.07885418166);
node_list.push_back(node26969);
Node node26970(26970,37.54227237267,127.07921128166);
node_list.push_back(node26970);
Node node26971(26971,37.54230547268,127.07879558166);
node_list.push_back(node26971);
Node node26972(26972,37.54227717268,127.07915548166);
node_list.push_back(node26972);
Node node26973(26973,37.54229727268,127.07892348166);
node_list.push_back(node26973);
Node node26974(26974,37.54230057268,127.07887738166);
node_list.push_back(node26974);
Node node26975(26975,37.54231637268,127.07864088166);
node_list.push_back(node26975);
Node node26976(26976,37.54227097267,127.07923398166);
node_list.push_back(node26976);
Node node26977(26977,37.54230387268,127.07881848166);
node_list.push_back(node26977);
Node node26978(26978,37.54227507268,127.07917758166);
node_list.push_back(node26978);
Node node26979(26979,37.54229517268,127.07894718166);
node_list.push_back(node26979);
Node node26980(26980,37.54229897268,127.07890038166);
node_list.push_back(node26980);
Node node26981(26981,37.54226297267,127.07930108166);
node_list.push_back(node26981);
Node node26982(26982,37.54231437268,127.07866418166);
node_list.push_back(node26982);
Node node26983(26983,37.54226877267,127.07925698166);
node_list.push_back(node26983);
Node node26984(26984,37.54228897268,127.07902918166);
node_list.push_back(node26984);
Node node26985(26985,37.54231957268,127.07860398166);
node_list.push_back(node26985);
Node node26986(26986,37.54225837267,127.07937228166);
node_list.push_back(node26986);
Node node26987(26987,37.54229377268,127.07897068166);
node_list.push_back(node26987);
Node node26988(26988,37.54231007268,127.07873678166);
node_list.push_back(node26988);
Node node26989(26989,37.54232195537,127.07858182423);
node_list.push_back(node26989);
Node node27115(27115,37.54161591801,127.07844644328);
node_list.push_back(node27115);
Node node27116(27116,37.54160604947,127.07845963875);
node_list.push_back(node27116);
Node node27117(27117,37.54168261856,127.07845404328);
node_list.push_back(node27117);
Node node27118(27118,37.54125904663,127.07840973875);
node_list.push_back(node27118);
Node node27119(27119,37.54124991503,127.07839374328);
node_list.push_back(node27119);
Node node27120(27120,37.54141601639,127.07841704328);
node_list.push_back(node27120);
Node node27121(27121,37.5415601491,127.07845253875);
node_list.push_back(node27121);
Node node27122(27122,37.54119941462,127.07838734328);
node_list.push_back(node27122);
Node node27123(27123,37.54136754752,127.07842433875);
node_list.push_back(node27123);
Node node27124(27124,37.54122891486,127.07839094328);
node_list.push_back(node27124);
Node node27125(27125,37.5413280472,127.07842073875);
node_list.push_back(node27125);
Node node27126(27126,37.541665045,127.07846710347);
node_list.push_back(node27126);
Node node27127(27127,37.54133931576,127.07840714328);
node_list.push_back(node27127);
Node node27128(27128,37.54112684555,127.07839063875);
node_list.push_back(node27128);
Node node27129(27129,37.54111811396,127.07837534328);
node_list.push_back(node27129);
Node node27130(27130,37.54162424962,127.07846223875);
node_list.push_back(node27130);
Node node27131(27131,37.54118911453,127.07838584328);
node_list.push_back(node27131);
Node node27132(27132,37.54143454807,127.07843483875);
node_list.push_back(node27132);
Node node27133(27133,37.54159821787,127.07844324328);
node_list.push_back(node27133);
Node node27134(27134,37.54157874925,127.07845543875);
node_list.push_back(node27134);
Node node27135(27135,37.54139711623,127.07841444328);
node_list.push_back(node27135);
Node node27136(27136,37.54138654768,127.07842703875);
node_list.push_back(node27136);
Node node27137(27137,37.54157971772,127.07844084328);
node_list.push_back(node27137);
Node node27138(27138,37.54134814736,127.07842263875);
node_list.push_back(node27138);
Node node27139(27139,37.54168563775,127.07846830348);
node_list.push_back(node27139);
Node node27140(27140,37.54153231733,127.07843354328);
node_list.push_back(node27140);
Node node27141(27141,37.54114644571,127.07839393875);
node_list.push_back(node27141);
Node node27142(27142,37.54164384978,127.07846503875);
node_list.push_back(node27142);
Node node27143(27143,37.54138751615,127.07841244328);
node_list.push_back(node27143);
Node node27144(27144,37.54145324822,127.07843903875);
node_list.push_back(node27144);
Node node27145(27145,37.54151271717,127.07843154328);
node_list.push_back(node27145);
Node node27146(27146,37.54166441841,127.07845284328);
node_list.push_back(node27146);
Node node27147(27147,37.54140574783,127.07843013875);
node_list.push_back(node27147);
Node node27148(27148,37.54127961527,127.07839774328);
node_list.push_back(node27148);
Node node27149(27149,37.54120834622,127.07840303875);
node_list.push_back(node27149);
Node node27150(27150,37.54160701794,127.07844504328);
node_list.push_back(node27150);
Node node27151(27151,37.54116754589,127.07839713875);
node_list.push_back(node27151);
Node node27152(27152,37.54143551655,127.07842024328);
node_list.push_back(node27152);
Node node27153(27153,37.5415117487,127.07844613875);
node_list.push_back(node27153);
Node node27154(27154,37.54113761411,127.07837784328);
node_list.push_back(node27154);
Node node27155(27155,37.54147324839,127.07844123875);
node_list.push_back(node27155);
Node node27156(27156,37.54127864679,127.07841233875);
node_list.push_back(node27156);
Node node27157(27157,37.54132901568,127.07840614328);
node_list.push_back(node27157);
Node node27158(27158,37.54142444799,127.07843323875);
node_list.push_back(node27158);
Node node27159(27159,37.54121901478,127.07838964328);
node_list.push_back(node27159);
Node node27160(27160,37.54122794638,127.07840553875);
node_list.push_back(node27160);
Node node27161(27161,37.54110821388,127.07837384328);
node_list.push_back(node27161);
Node node27162(27162,37.54118814605,127.07840043875);
node_list.push_back(node27162);
Node node27163(27163,37.54162521809,127.07844764328);
node_list.push_back(node27163);
Node node27164(27164,37.54153134886,127.07844813875);
node_list.push_back(node27164);
Node node27165(27165,37.54115801428,127.07838084328);
node_list.push_back(node27165);
Node node27166(27166,37.54149264854,127.07844323875);
node_list.push_back(node27166);
Node node27167(27167,37.5412093147,127.07838844328);
node_list.push_back(node27167);
Node node27168(27168,37.54129794695,127.07841543875);
node_list.push_back(node27168);
Node node27169(27169,37.541368516,127.07840974328);
node_list.push_back(node27169);
Node node27170(27170,37.5415972494,127.07845783875);
node_list.push_back(node27170);
Node node27171(27171,37.54114741419,127.07837934328);
node_list.push_back(node27171);
Node node27172(27172,37.54124894655,127.07840833875);
node_list.push_back(node27172);
Node node27173(27173,37.54130861551,127.07840274328);
node_list.push_back(node27173);
Node node27174(27174,37.54131861559,127.07840454328);
node_list.push_back(node27174);
Node node27175(27175,37.54112781404,127.07837604328);
node_list.push_back(node27175);
Node node27176(27176,37.54155074902,127.07845123875);
node_list.push_back(node27176);
Node node27177(27177,37.54126001511,127.07839514328);
node_list.push_back(node27177);
Node node27178(27178,37.54135794744,127.07842363875);
node_list.push_back(node27178);
Node node27179(27179,37.54140671631,127.07841554328);
node_list.push_back(node27179);
Node node27180(27180,37.54131764711,127.07841913875);
node_list.push_back(node27180);
Node node27181(27181,37.54123931494,127.07839224328);
node_list.push_back(node27181);
Node node27182(27182,37.54111714547,127.07838993875);
node_list.push_back(node27182);
Node node27183(27183,37.54156111756,127.07843794328);
node_list.push_back(node27183);
Node node27184(27184,37.54161494954,127.07846103875);
node_list.push_back(node27184);
Node node27185(27185,37.54109751378,127.07837244328);
node_list.push_back(node27185);
Node node27186(27186,37.54126864671,127.07841103875);
node_list.push_back(node27186);
Node node27187(27187,37.54134911584,127.07840804328);
node_list.push_back(node27187);
Node node27188(27188,37.54156954917,127.07845403875);
node_list.push_back(node27188);
Node node27189(27189,37.5414542167,127.07842444328);
node_list.push_back(node27189);
Node node27190(27190,37.5413768476,127.07842583875);
node_list.push_back(node27190);
Node node27191(27191,37.54148381694,127.07842754328);
node_list.push_back(node27191);
Node node27192(27192,37.54133834728,127.07842173875);
node_list.push_back(node27192);
Node node27193(27193,37.54167497677,127.07846790347);
node_list.push_back(node27193);
Node node27194(27194,37.54158891779,127.07844234328);
node_list.push_back(node27194);
Node node27195(27195,37.54113664563,127.07839243875);
node_list.push_back(node27195);
Node node27196(27196,37.54167381849,127.07845364328);
node_list.push_back(node27196);
Node node27197(27197,37.5416340497,127.07846373875);
node_list.push_back(node27197);
Node node27198(27198,37.54144451662,127.07842214328);
node_list.push_back(node27198);
Node node27199(27199,37.54144354814,127.07843673875);
node_list.push_back(node27199);
Node node27200(27200,37.54157051764,127.07843944328);
node_list.push_back(node27200);
Node node27201(27201,37.5410965453,127.07838703875);
node_list.push_back(node27201);
Node node27202(27202,37.54149361702,127.07842864328);
node_list.push_back(node27202);
Node node27203(27203,37.54139614775,127.07842903875);
node_list.push_back(node27203);
Node node27204(27204,37.54152231725,127.07843254328);
node_list.push_back(node27204);
Node node27205(27205,37.54119844614,127.07840193875);
node_list.push_back(node27205);
Node node27206(27206,37.54169603289,127.07847034458);
node_list.push_back(node27206);
Node node27207(27207,37.54128971535,127.07839944328);
node_list.push_back(node27207);
Node node27208(27208,37.5411570458,127.07839543875);
node_list.push_back(node27208);
Node node27209(27209,37.54117891445,127.07838424328);
node_list.push_back(node27209);
Node node27210(27210,37.54165354986,127.07846663875);
node_list.push_back(node27210);
Node node27211(27211,37.54163501817,127.07844914328);
node_list.push_back(node27211);
Node node27212(27212,37.5414633483,127.07844023875);
node_list.push_back(node27212);
Node node27213(27213,37.54116851436,127.07838254328);
node_list.push_back(node27213);
Node node27214(27214,37.54110724539,127.07838843875);
node_list.push_back(node27214);
Node node27215(27215,37.54141504791,127.07843163875);
node_list.push_back(node27215);
Node node27216(27216,37.5412180463,127.07840423875);
node_list.push_back(node27216);
Node node27217(27217,37.54155171749,127.07843664328);
node_list.push_back(node27217);
Node node27218(27218,37.54117794597,127.07839883875);
node_list.push_back(node27218);
Node node27219(27219,37.54137781607,127.07841124328);
node_list.push_back(node27219);
Node node27220(27220,37.54152134878,127.07844713875);
node_list.push_back(node27220);
Node node27221(27221,37.54150301709,127.07843014328);
node_list.push_back(node27221);
Node node27222(27222,37.54148284846,127.07844213875);
node_list.push_back(node27222);
Node node27223(27223,37.54126961519,127.07839644328);
node_list.push_back(node27223);
Node node27224(27224,37.54128874688,127.07841403875);
node_list.push_back(node27224);
Node node27225(27225,37.54154181741,127.07843514328);
node_list.push_back(node27225);
Node node27226(27226,37.54142541646,127.07841864328);
node_list.push_back(node27226);
Node node27227(27227,37.54158794932,127.07845693875);
node_list.push_back(node27227);
Node node27228(27228,37.54147421686,127.07842664328);
node_list.push_back(node27228);
Node node27229(27229,37.54123834646,127.07840683875);
node_list.push_back(node27229);
Node node27230(27230,37.54164481825,127.07845044328);
node_list.push_back(node27230);
Node node27231(27231,37.54165451833,127.07845204328);
node_list.push_back(node27231);
Node node27232(27232,37.54154084894,127.07844973875);
node_list.push_back(node27232);
Node node27233(27233,37.54135891592,127.07840904328);
node_list.push_back(node27233);
Node node27234(27234,37.5416948746,127.07845541383);
node_list.push_back(node27234);
Node node27235(27235,37.54150204862,127.07844473875);
node_list.push_back(node27235);
Node node27236(27236,37.54146431678,127.07842564328);
node_list.push_back(node27236);
Node node27237(27237,37.54130764703,127.07841733875);
node_list.push_back(node27237);
Node node27238(27238,37.54129891543,127.07840084328);
node_list.push_back(node27238);
Node node27383(27383,37.54101550161,127.07914871087);
node_list.push_back(node27383);
Node node27384(27384,37.54100179384,127.07915715142);
node_list.push_back(node27384);
Node node27385(27385,37.54106720248,127.07851351087);
node_list.push_back(node27385);
Node node27386(27386,37.54101960168,127.07910441087);
node_list.push_back(node27386);
Node node27387(27387,37.54104439457,127.07865435142);
node_list.push_back(node27387);
Node node27388(27388,37.54105239471,127.07853645142);
node_list.push_back(node27388);
Node node27389(27389,37.54104080203,127.07887841087);
node_list.push_back(node27389);
Node node27390(27390,37.54105529476,127.07848795142);
node_list.push_back(node27390);
Node node27391(27391,37.54101419406,127.07902355142);
node_list.push_back(node27391);
Node node27392(27392,37.54107130255,127.07845441087);
node_list.push_back(node27392);
Node node27393(27393,37.54101200155,127.07921671087);
node_list.push_back(node27393);
Node node27394(27394,37.54105469475,127.07849955142);
node_list.push_back(node27394);
Node node27395(27395,37.54104610212,127.07881941087);
node_list.push_back(node27395);
Node node27396(27396,37.54103469441,127.07879445142);
node_list.push_back(node27396);
Node node27397(27397,37.54100649392,127.07910255142);
node_list.push_back(node27397);
Node node27398(27398,37.5410626024,127.07858651087);
node_list.push_back(node27398);
Node node27399(27399,37.54101430159,127.07916881087);
node_list.push_back(node27399);
Node node27400(27400,37.5410338944,127.07880625142);
node_list.push_back(node27400);
Node node27401(27401,37.54106550245,127.07853831087);
node_list.push_back(node27401);
Node node27402(27402,37.54103470193,127.07894781087);
node_list.push_back(node27402);
Node node27403(27403,37.54101189401,127.07905775142);
node_list.push_back(node27403);
Node node27404(27404,37.54104949466,127.07858465142);
node_list.push_back(node27404);
Node node27405(27405,37.541038802,127.07890171087);
node_list.push_back(node27405);
Node node27406(27406,37.5410398945,127.07872415142);
node_list.push_back(node27406);
Node node27407(27407,37.54102569426,127.07889985142);
node_list.push_back(node27407);
Node node27408(27408,37.54105710231,127.07866771087);
node_list.push_back(node27408);
Node node27409(27409,37.54103919449,127.07873595142);
node_list.push_back(node27409);
Node node27410(27410,37.54102730181,127.07902541087);
node_list.push_back(node27410);
Node node27411(27411,37.54106049484,127.07842775142);
node_list.push_back(node27411);
Node node27412(27412,37.54100239385,127.07914685142);
node_list.push_back(node27412);
Node node27413(27413,37.54106030236,127.07861031087);
node_list.push_back(node27413);
Node node27414(27414,37.54101270156,127.07918861087);
node_list.push_back(node27414);
Node node27415(27415,37.54105929482,127.07844005142);
node_list.push_back(node27415);
Node node27416(27416,37.54103250189,127.07896971087);
node_list.push_back(node27416);
Node node27417(27417,37.54104149452,127.07870115142);
node_list.push_back(node27417);
Node node27418(27418,37.54099889379,127.07921485142);
node_list.push_back(node27418);
Node node27419(27419,37.54103650196,127.07892501087);
node_list.push_back(node27419);
Node node27420(27420,37.54101599409,127.07900135142);
node_list.push_back(node27420);
Node node27421(27421,37.54105629477,127.07847645142);
node_list.push_back(node27421);
Node node27422(27422,37.54105550228,127.07869161087);
node_list.push_back(node27422);
Node node27423(27423,37.54101509408,127.07901205142);
node_list.push_back(node27423);
Node node27424(27424,37.54102560178,127.07904831087);
node_list.push_back(node27424);
Node node27425(27425,37.54102069417,127.07895665142);
node_list.push_back(node27425);
Node node27426(27426,37.54105840233,127.07863211087);
node_list.push_back(node27426);
Node node27427(27427,37.54104239454,127.07868975142);
node_list.push_back(node27427);
Node node27428(27428,37.54103020185,127.07899251087);
node_list.push_back(node27428);
Node node27429(27429,37.54101809413,127.07897925142);
node_list.push_back(node27429);
Node node27430(27430,37.54102769429,127.07887655142);
node_list.push_back(node27430);
Node node27431(27431,37.54105030219,127.07876101087);
node_list.push_back(node27431);
Node node27432(27432,37.54104069451,127.07871245142);
node_list.push_back(node27432);
Node node27433(27433,37.54105380225,127.07871431087);
node_list.push_back(node27433);
Node node27434(27434,37.54101790165,127.07911551087);
node_list.push_back(node27434);
Node node27435(27435,37.54103099435,127.07884115142);
node_list.push_back(node27435);
Node node27436(27436,37.54106940251,127.07847831087);
node_list.push_back(node27436);
Node node27437(27437,37.54102400175,127.07907111087);
node_list.push_back(node27437);
Node node27438(27438,37.54100059382,127.07917715142);
node_list.push_back(node27438);
Node node27439(27439,37.54104410209,127.07884301087);
node_list.push_back(node27439);
Node node27440(27440,37.54101939415,127.07896785142);
node_list.push_back(node27440);
Node node27441(27441,37.54107510261,127.07841761087);
node_list.push_back(node27441);
Node node27442(27442,37.54102820182,127.07901391087);
node_list.push_back(node27442);
Node node27443(27443,37.54101329404,127.07903505142);
node_list.push_back(node27443);
Node node27444(27444,37.54104840216,127.07878461087);
node_list.push_back(node27444);
Node node27445(27445,37.54101709411,127.07899065142);
node_list.push_back(node27445);
Node node27446(27446,37.54105230223,127.07873781087);
node_list.push_back(node27446);
Node node27447(27447,37.54101650162,127.07913771087);
node_list.push_back(node27447);
Node node27448(27448,37.54105329472,127.07852395142);
node_list.push_back(node27448);
Node node27449(27449,37.54106780249,127.07850141087);
node_list.push_back(node27449);
Node node27450(27450,37.54102150171,127.07909371087);
node_list.push_back(node27450);
Node node27451(27451,37.54103209437,127.07882915142);
node_list.push_back(node27451);
Node node27452(27452,37.54102159419,127.07894595142);
node_list.push_back(node27452);
Node node27453(27453,37.54104180205,127.07886651087);
node_list.push_back(node27453);
Node node27454(27454,37.54100969398,127.07908075142);
node_list.push_back(node27454);
Node node27455(27455,37.54104719462,127.07860845142);
node_list.push_back(node27455);
Node node27456(27456,37.54107240256,127.07844191087);
node_list.push_back(node27456);
Node node27457(27457,37.54101210155,127.07920791087);
node_list.push_back(node27457);
Node node27458(27458,37.54100839396,127.07909185142);
node_list.push_back(node27458);
Node node27459(27459,37.54104700214,127.07880811087);
node_list.push_back(node27459);
Node node27460(27460,37.54105099468,127.07856055142);
node_list.push_back(node27460);
Node node27461(27461,37.54101249402,127.07904645142);
node_list.push_back(node27461);
Node node27462(27462,37.54106350241,127.07857431087);
node_list.push_back(node27462);
Node node27463(27463,37.5410149016,127.07915901087);
node_list.push_back(node27463);
Node node27464(27464,37.54105039467,127.07857245142);
node_list.push_back(node27464);
Node node27465(27465,37.54106640246,127.07852581087);
node_list.push_back(node27465);
Node node27466(27466,37.54103570195,127.07893661087);
node_list.push_back(node27466);
Node node27467(27467,37.54102659427,127.07888845142);
node_list.push_back(node27467);
Node node27468(27468,37.54100119383,127.07916695142);
node_list.push_back(node27468);
Node node27469(27469,37.54103970201,127.07889031087);
node_list.push_back(node27469);
Node node27470(27470,37.54100389388,127.07912485142);
node_list.push_back(node27470);
Node node27471(27471,37.54104399457,127.07866585142);
node_list.push_back(node27471);
Node node27472(27472,37.54105750231,127.07865621087);
node_list.push_back(node27472);
Node node27473(27473,37.54100339387,127.07913585142);
node_list.push_back(node27473);
Node node27474(27474,37.54104520211,127.07883101087);
node_list.push_back(node27474);
Node node27475(27475,37.5409992938,127.07919605142);
node_list.push_back(node27475);
Node node27476(27476,37.54105409474,127.07851165142);
node_list.push_back(node27476);
Node node27477(27477,37.54106130238,127.07859881087);
node_list.push_back(node27477);
Node node27478(27478,37.54101370158,127.07917901087);
node_list.push_back(node27478);
Node node27479(27479,37.5409989938,127.07920605142);
node_list.push_back(node27479);
Node node27480(27480,37.54106470243,127.07855061087);
node_list.push_back(node27480);
Node node27481(27481,37.54103380192,127.07895851087);
node_list.push_back(node27481);
Node node27482(27482,37.54105729479,127.07846465142);
node_list.push_back(node27482);
Node node27483(27483,37.54103299438,127.07881755142);
node_list.push_back(node27483);
Node node27484(27484,37.54103750198,127.07891351087);
node_list.push_back(node27484);
Node node27485(27485,37.54103609443,127.07877095142);
node_list.push_back(node27485);
Node node27486(27486,37.541010894,127.07906925142);
node_list.push_back(node27486);
Node node27487(27487,37.5410564023,127.07867981087);
node_list.push_back(node27487);
Node node27488(27488,37.54103529442,127.07878275142);
node_list.push_back(node27488);
Node node27489(27489,37.54102640179,127.07903691087);
node_list.push_back(node27489);
Node node27490(27490,37.54102439424,127.07891165142);
node_list.push_back(node27490);
Node node27491(27491,37.54105950235,127.07862121087);
node_list.push_back(node27491);
Node node27492(27492,37.54102869431,127.07886465142);
node_list.push_back(node27492);
Node node27493(27493,37.54103120187,127.07898111087);
node_list.push_back(node27493);
Node node27494(27494,37.54103819447,127.07874725142);
node_list.push_back(node27494);
Node node27495(27495,37.54105819481,127.07845255142);
node_list.push_back(node27495);
Node node27496(27496,37.54105130221,127.07874911087);
node_list.push_back(node27496);
Node node27497(27497,37.54104479458,127.07864225142);
node_list.push_back(node27497);
Node node27498(27498,37.54100479389,127.07911365142);
node_list.push_back(node27498);
Node node27499(27499,37.54105460226,127.07870301087);
node_list.push_back(node27499);
Node node27500(27500,37.54106199487,127.07841575142);
node_list.push_back(node27500);
Node node27501(27501,37.54107040253,127.07846651087);
node_list.push_back(node27501);
Node node27502(27502,37.54102500177,127.07905961087);
node_list.push_back(node27502);
Node node27503(27503,37.54105159469,127.07854875142);
node_list.push_back(node27503);
Node node27504(27504,37.54105790232,127.07864411087);
node_list.push_back(node27504);
Node node27505(27505,37.54102339422,127.07892315142);
node_list.push_back(node27505);
Node node27506(27506,37.54107590262,127.07840819308);
node_list.push_back(node27506);
Node node27507(27507,37.54102910184,127.07900321087);
node_list.push_back(node27507);
Node node27508(27508,37.54104639461,127.07861935142);
node_list.push_back(node27508);
Node node27509(27509,37.54104920217,127.07877281087);
node_list.push_back(node27509);
Node node27510(27510,37.54103719445,127.07875915142);
node_list.push_back(node27510);
Node node27511(27511,37.54105300224,127.07872601087);
node_list.push_back(node27511);
Node node27512(27512,37.54101700163,127.07912671087);
node_list.push_back(node27512);
Node node27513(27513,37.5410225942,127.07893475142);
node_list.push_back(node27513);
Node node27514(27514,37.5410684025,127.07848981087);
node_list.push_back(node27514);
Node node27515(27515,37.54102280173,127.07908261087);
node_list.push_back(node27515);
Node node27516(27516,37.54104819464,127.07859695142);
node_list.push_back(node27516);
Node node27517(27517,37.54104300207,127.07885481087);
node_list.push_back(node27517);
Node node27518(27518,37.54102989433,127.07885295142);
node_list.push_back(node27518);
Node node27519(27519,37.54099959381,127.07918675142);
node_list.push_back(node27519);
Node node27520(27520,37.54107360258,127.07842961087);
node_list.push_back(node27520);
Node node27521(27521,37.54101240156,127.07919791087);
node_list.push_back(node27521);
Node node27522(27522,37.54104329455,127.07867795142);
node_list.push_back(node27522);
Node node27523(27523,37.54104780215,127.07879631087);
node_list.push_back(node27523);
Node node27524(27524,37.54106287758,127.07840559399);
node_list.push_back(node27524);
Node node27525(27525,37.54104529459,127.07863025142);
node_list.push_back(node27525);
Node node27526(27526,37.54106410242,127.07856241087);
node_list.push_back(node27526);
Node node27528(27528,37.5410828571,127.07837294754);
node_list.push_back(node27528);
Node node27529(27529,37.54107461588,127.07837797668);
node_list.push_back(node27529);
Node node27530(27530,37.54106690634,127.07838937607);
node_list.push_back(node27530);
Node node27531(27531,37.54108524972,127.07839138772);
node_list.push_back(node27531);
Node node27532(27532,37.5410799328,127.07839742269);
node_list.push_back(node27532);

}

vector<Node*> aStar(Node* start, Node* end) {//A* 알고리즘, 시작점과 끝점을 인자로
    priority_queue<Node*, vector<Node*>, CompareNode> openSet;//오픈리스트
    unordered_set<Node*> set;//오픈 리스트에 특정 객체 존재 여부 확인용, 우선순위큐는 특정 값 존재 여부를 알수 없기 때문
    unordered_set<Node*> closedSet;//닫힌리스트

    start->g = 0; //시작점이니까 0
    start->h = heuristic(start, end); //h 계산
    start->f = start->g + start->h;//f - g+h
    openSet.push(start);//오픈리스트(우선순위 큐)에 추가
    set.insert(start);//나중에 오픈리스트에 값이 존재하는지 알아야해서 넣는것 뿐

    while (!openSet.empty()) {//오픈리스트가 빌때 꺄지
        Node* current = openSet.top();//f 값이 가장 작은 노드
        openSet.pop();//f값이 가장 작은 노드를 빼준다
        set.erase(current);//여기서도 지워준다
        closedSet.insert(current);//닫힌 리스트에 현재 노드 추가

        if (current == end) {//목적지에 도달했으면 끝내자
            vector<Node*> path;//결과 리스트
            while (current != start) {//시작점 전까지 parent따라서 경로 저장
                path.push_back(current);
                current = current->parent;
            }
            path.push_back(start);//시작점도 추가
            reverse(path.begin(), path.end());//시작부터 끝까지 정렬
            return path;//최단 경로 리턴
        }
        
        

        for (Node* neighbor : current->neighbors) {//현재 노드의 모든 이웃에 대해서
            if (closedSet.find(neighbor) != closedSet.end()) {//neighbor가 닫힌 리스트에 있으면 건너뛰자
                continue;
            }
            if(set.find(neighbor) == set.end()){//이웃이 열린 리스트에 포함되어 있지 않을때
                openSet.push(neighbor);//오픈 리스트에 추가
                set.insert(neighbor);//set에 추가
                neighbor->g = current->g + getDistance(neighbor,current);//이웃의 g값 = 현재 노드 g + 거리
                neighbor->f = neighbor->g + neighbor->h;//f값 업데이트
                neighbor->parent = current;//부모 노드 업데이트
            }
            else{//이웃이 열린 리스트에 포함되어 있을때
            double distance = getDistance(neighbor,current);//이웃 노드와의 거리
            if (neighbor->f  > current->g + distance + neighbor->h) { //새로운 경로로 계산된 f값이 기존 f보다 작으면 루트 변경
                neighbor->g = current->g + distance;//g값 업데이트
                neighbor->f = neighbor->g + neighbor->h;//f 업데이트
                neighbor->parent = current;//부모 업데이트
            }
            }
        }
        
    }

    return vector<Node*>(); // 경로를 찾지 못한 경우 빈 리스트 반환
}

int main() {
    vector<Node> node_list;
    init_nodes(node_list);
    draw_graph(node_list);
    
    vector<Node*> global_path;
    global_path = Astar(node_list[0],node_list[node_list.size()-1]);//예시
    
    /*
    ifstream file("node.csv");
    string line;
    while (getline(file, line)) {
        double lat, lon;
        size_t start = line.find('(') + 1;  // '(' 다음 위치를 찾습니다.
        size_t end = line.find(')');        // ')' 위치를 찾습니다.
        
        if (start != string::npos && end != string::npos && end > start) {
            string coordinates = line.substr(start, end - start);  // 위도와 경도 추출
            replace(coordinates.begin(), coordinates.end(), ',', ' ');  // 쉼표를 공백으로 대체

            stringstream ss(coordinates);
            if (ss >> lat >> lon) {
                Node newNode(lat, lon); // 새로운 Node 객체 생성
                node_list.push_back(newNode);  // 새 노드를 벡터에 추가
            }
        }
    }

    file.close();  // 파일 닫기

    // 파일 파싱 결과 출력
    for (const Node& node : node_list) {
        cout << "Node: (" << node.x << ", " << node.y << ")" << endl;
    }

    // 이제 node_list를 사용하여 필요한 작업을 수행할 수 있습니다. 
    */
   return 0;
}
