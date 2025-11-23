#pragma once
#include<unordered_map>
#include<string>
#include<set>

struct SRTP{
    int id;
    int source;
    int target;
    std::string mode;
    std::unordered_map<int,bool> forbidden_nodes;
    std::set<std::pair<int,int>> forbidden_edges;
    std::unordered_map<std::string,bool> forbidden_road_types = {
    {"expressway",false},
    {"primary",false},
    {"secondary",false},
    {"tertiary",false},
    {"local",false}
    };
//    SRTP(int id, int source, int target, const std::string& mode,
//          const std::set<std::pair<int,int>>& forbidden_edges = {})
//         : id(id), source(source), target(target), mode(mode),
//           forbidden_edges(forbidden_edges) {}
    };