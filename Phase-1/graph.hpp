#pragma once
#include<unordered_map>
#include<vector>
#include<iostream>
#include<utility>
#include<queue>
#include<set>
#include<list>
#include "srtp.hpp"
#include "knn.h"
#include "kdt.h"
#include "common/nlohmann/json.hpp"
#define M_PI 3.14159265358979323846
using json = nlohmann::json;
///////////////////////////////////////////////////////////////
struct Node{
    int id;
    double lat,lon;
    std::set<std::string> poi;
    Node(int i, double la, double lo, const json& pois) 
        : id(i), lat(la), lon(lo) {
        for (const auto& p : pois) {
            if(p != " ")
            poi.insert(p);
        }
    }
};
struct Edge{
    int id;
    int u,v;
    double length;
    double avg_time;
    std::vector<double>speed_profile;
    bool oneway;
    std::string road_type;
    bool active = true;
    Edge(){}
    Edge(int i, int u_, int v_, double len, double time, 
         const json& speed, bool one, const std::string& type) 
        : id(i), u(u_), v(v_), length(len), avg_time(time), 
          oneway(one), road_type(type) {
        for (const auto& s : speed) {
            speed_profile.push_back(s);
        }
    }
};

class Graph
{
private:
int N;
int current_token;
bool alt_initialized = false;
std::vector<Node*> Nodes;
std::vector<Edge*> Edges;
std::vector<int> edge_visited_token;
std::map<std::pair<int,int>,Edge*> edge_lookup;
std::vector<std::vector<Edge*>> adj;
std::vector<std::vector<Edge*>> adj_r;
kdtree* kd_tree;
std::vector<int>  landmarks;
std::vector<std::vector<double>> dist_from_landmark;
std::vector<std::vector<double>> dist_to_landmark;
public:
    Graph(const json& graph_json);
    json process_query(const json& query);
    json handleRemoveEdge(const json& query);
    json handleModifyEdge(const json& query);
    //phase1
    std::vector<int> handleShortesPath(SRTP sp,bool &possible,double & mtbd); //task1//for mode =distance
    std::vector<int> A_star_time(SRTP& sp,bool &possible,double & mtbd); //task1//for mode = time
    std::vector<int> handleKnn(KNN knn); //task2
    //phase2
    // std::list<std::pair<double,std::vector<int>>> handle_K_ShortestPaths(SRTP sp, int K);//task1
    // std::vector<std::pair<std::vector<int>,double>> k_shortest_paths_heustirics(KSP_H ksp);//task2
    // double weighted_a_star(int source, int target,double epsilon,double timeout);//task3
    //helpers
    std::vector<Node*> getNodes(){ return Nodes; }
    //phase1//task1 helpers
    std::vector<double> euclidean_distance_to(int target);
    double get_haversine_dist(int u_id,int v_id);
    //phase2//task2 helpers
    //std::vector<int> A_starimpl(SRTP sp,bool &possible,double & mtbd);
    //phase2//task3 helpers
    // void precompute_alt();
    // double get_alt_heuristic(int u, int target);
};