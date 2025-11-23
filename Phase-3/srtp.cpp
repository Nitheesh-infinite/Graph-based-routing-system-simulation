#include "graph.hpp"
#include "srtp.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

const double INF = std::numeric_limits<double>::infinity();

// Phase 3: We use avg_time, so calc_time is strictly a fallback
double calc_time(double curr_time, double length, const std::vector<double>& speed_profile)
{
    // Minimal implementation for compilation safety
    if(speed_profile.empty()) return curr_time + (length/10.0);
    return curr_time + (length/13.0); // approx avg speed
}

double Graph::get_haversine_dist(int u_id, int v_id) {
    const double R = 6371000.0; // Meters
    if (u_id < 0 || u_id >= Nodes.size() || v_id < 0 || v_id >= Nodes.size()) return 1e9; 
    
    const Node* node_u = Nodes[u_id];
    const Node* node_v = Nodes[v_id];
    
    auto to_rad = [](double degree) { return degree * M_PI / 180.0; };
    double lat1 = to_rad(node_u->lat);
    double lon1 = to_rad(node_u->lon);
    double lat2 = to_rad(node_v->lat);
    double lon2 = to_rad(node_v->lon);
    
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
    return R * c;
}

// Phase 3 Optimized A* using Average Time
std::vector<int> Graph::A_star_time(const SRTP& sp, bool &possible, double & mtbd) {
    int n = Nodes.size();
    possible = false;
    
    // Heuristic Constant: 40 m/s (~144 km/h) to be admissible
    const double MAX_SPEED_MPS = 40.0; 

    using P = std::pair<double, int>; // {F-Score, Node}
    std::priority_queue<P, std::vector<P>, std::greater<P>> min_heap;

    std::vector<double> g_score(n, INF);
    std::vector<int> parent(n, -1);

    if (sp.source < 0 || sp.source >= n || sp.target < 0 || sp.target >= n) return {};
    if (sp.forbidden_nodes.count(sp.source)) return {}; 

    g_score[sp.source] = 0.0;
    double h_start = get_haversine_dist(sp.source, sp.target) / MAX_SPEED_MPS;
    min_heap.push({0.0 + h_start, sp.source});

    while(!min_heap.empty()){
        auto [f, u] = min_heap.top();
        min_heap.pop();

        // Lazy deletion optimization
        if (f > g_score[u] + (get_haversine_dist(u, sp.target)/MAX_SPEED_MPS) + 1.0) continue;

        if (u == sp.target) {
            possible = true;
            break;
        }

        for(auto &e: adj[u]){
            if (!e->active) continue;
            
            // Constraint Checks
            auto it_type = sp.forbidden_road_types.find(e->road_type);
            if (it_type != sp.forbidden_road_types.end() && it_type->second) continue;
            if (sp.forbidden_nodes.count(e->v)) continue;

            // Phase 3: Use Average Time directly
            double edge_cost = e->avg_time;
            if(edge_cost <= 0) edge_cost = e->length / 13.8; // Fallback

            double new_g = g_score[u] + edge_cost;

            if (new_g < g_score[e->v]) {
                g_score[e->v] = new_g;
                parent[e->v] = u;
                
                double h = get_haversine_dist(e->v, sp.target) / MAX_SPEED_MPS;
                min_heap.push({new_g + h, e->v}); 
            }
        }
    }

    std::vector<int> path;
    if (!possible) {
        mtbd = INF;
        return path;
    }

    mtbd = g_score[sp.target];
    for (int cur = sp.target; cur != -1; cur = parent[cur]) path.push_back(cur);
    std::reverse(path.begin(), path.end());
    return path;
}

// Keeps legacy handleShortesPath for compilation, simplified
std::vector<int> Graph::handleShortesPath(const SRTP& sp, bool &possible, double &mtbd) {
    // Re-use A_star_time for simplicity in Phase 3 or implement Dijkstra here if strictly needed
    // For Phase 3 fleet, only A_star_time is used.
    return A_star_time(sp, possible, mtbd); 
}



std::vector<double> Graph::euclidean_distance_to(int target) {
    return {}; // Placeholder
}