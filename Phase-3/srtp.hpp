#pragma once
#include<unordered_map>
#include<string>
#include<set>
#include<vector>

struct SRTP{
    int id = -1;
    int source = -1;
    int target = -1;
    std::string mode = "time";
    std::unordered_map<int,bool> forbidden_nodes;
    std::set<std::pair<int,int>> forbidden_edges;
    std::unordered_map<std::string,bool> forbidden_road_types = {
        {"expressway",false},
        {"primary",false},
        {"secondary",false},
        {"tertiary",false},
        {"local",false}
    };
};