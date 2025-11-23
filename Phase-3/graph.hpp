#pragma once
#include<unordered_map>
#include<vector>
#include<iostream>
#include<utility>
#include<queue>
#include<set>
#include<list>
#include<map>
#include "srtp.hpp"

#include "../common/nlohmann/json.hpp"

#define M_PI 3.14159265358979323846
using json = nlohmann::json;

struct Node{
    int id;
    double lat,lon;
    std::set<std::string> poi;
    Node(int i, double la, double lo, const json& pois) 
        : id(i), lat(la), lon(lo) {
        for (const auto& p : pois) {
            if(p != " ") poi.insert(p);
        }
    }
};

struct Edge{
    int id;
    int u,v;
    double length;
    double avg_time;
    std::vector<double> speed_profile;
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
    std::vector<Node*> Nodes;
    std::vector<Edge*> Edges;
    std::vector<int> edge_visited_token;
    std::vector<std::vector<Edge*>> adj;
    std::vector<std::vector<Edge*>> adj_r;
    
public:
    Graph(const json& graph_json);
    json process_query(const json& query);
    
    // Core Pathfinding (Implemented in srtp.cpp)
    std::vector<int> handleShortesPath(const SRTP& sp, bool &possible, double & mtbd); 
    std::vector<int> A_star_time(const SRTP& sp, bool &possible, double & mtbd); 
    
    // Helpers
   
    std::vector<Node*> getNodes(){ return Nodes; }
    std::vector<double> euclidean_distance_to(int target);
    double get_haversine_dist(int u_id, int v_id);
};