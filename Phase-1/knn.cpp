#include "graph.hpp"   
#include "knn.h"      
#include "kdt.h"        
#include <vector>                // for std::vector
#include <cmath>                 // distance computations
#include <algorithm>             // sort distances
#include <queue>                 // priority queue
#include <utility>               // pair

const double INF = 1e9;

std::vector<int> Graph::handleKnn(KNN knn){
    std::vector<int> final_ids;
    double lat = knn.lat;
    double lon = knn.lon;
    std::string poi = knn.poi;
    int k = knn.k;
    if(k == 0) return final_ids;
    if(knn.metric == "shortest_path"){
        kdnode target(0, lat, lon, {});
        std::vector<kdnode*> start_nodes = this->kd_tree->knn(this->kd_tree->getRoot(),target,1, "");

        if (start_nodes.empty()) return final_ids; 
        int id = start_nodes[0]->id;

        std::vector<double> distances(this->N, INF);
        std::vector<bool> visited(N,false);

        distances[id] = 0.0;
        std::priority_queue< std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<std::pair<double,int>>> q;
        q.push({0.0,id});
        int count = 0;
        while(!q.empty()){
            auto [du , o] = q.top();
            q.pop();
            if(visited[o]) continue;
            visited[o] = true;

            // Optimization: If current path is longer than found, skip
            if(du > distances[o]) continue;

            if(Nodes[o]->poi.count(poi)){
                final_ids.push_back(Nodes[o]->id);
                count++;
                if(count == k) break;
            }

            for( auto edge : this->adj[o] ){
                int v = edge->v;
                double w = edge->length;
                if(du + w < distances[v] && edge->active ){
                    distances[v] = du + w;
                    q.push({distances[v],v});
                }
            }
        }
    }

    else if(knn.metric == "euclidean"){
        std::vector<std::pair<double, int>> distances;
        for (int i = 0;i<this->N;i++) {
            double dx = lat-this->Nodes[i]->lat;
            double dy = lon-this->Nodes[i]->lon;
            double d = (dx*dx + dy*dy);
            if(Nodes[i]->poi.count(poi)){
                distances.push_back({d, this->Nodes[i]->id});
            }
        }
        std::sort(distances.begin(),distances.end());
        int a = (k >= distances.size())?distances.size():k;
        for(int i = 0;i<a;i++){
            final_ids.push_back(distances[i].second);
        }
    }

    return final_ids;
}

