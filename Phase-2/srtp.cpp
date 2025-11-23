#include "graph.hpp"
#include "srtp.hpp"
 const double INF = std::numeric_limits<double>::infinity();
double calc_time(double curr_time,double length, std::vector<double>speed_profile)
{
    while(length>0)
    {
        int t = static_cast<int>(curr_time/15);
        double next_slot = ((floor(curr_time/15)+1)*15);
        double time_left = next_slot - curr_time;
        double distance_possible = speed_profile[t]*time_left;
        if(length <= distance_possible)
        {
            return curr_time + length/(double)speed_profile[t];
        }
        length-=distance_possible;
        curr_time = next_slot;
    }
    return curr_time;
}

std::vector<double> Graph::euclidean_distance_to(int target)  {
    const double R = 6371.0; // km
    int n=Nodes.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    if (target < 0 || target >= n) return dist;
    const Node* t = Nodes[target];
    if (!t) return dist;

    auto rad = [](double deg){ return deg * M_PI / 180.0; };
    double lat2 = rad(t->lat), lon2 = rad(t->lon);
    
    for (int i = 0; i < n; ++i) {
        const Node* ni = Nodes[i];
        if (!ni) continue;
        double lat1 = rad(ni->lat), lon1 = rad(ni->lon);
        double dlat = lat2 - lat1, dlon = lon2 - lon1;
        double a = std::sin(dlat*0.5); a = a*a;
        double b = std::sin(dlon*0.5); b = b*b;
        double h = a + std::cos(lat1)*std::cos(lat2)*b;
        dist[i] = 2.0 * R * std::atan2(std::sqrt(h), std::sqrt(std::max(0.0, 1.0-h)));
    }
    return dist;
}
double Graph::get_haversine_dist(int u_id, int v_id) {
    
    const double R = 6371000.0; 
    if (u_id < 0 || u_id >= Nodes.size() || v_id < 0 || v_id >= Nodes.size()) {
        return 1e9; 
    }
    const Node& node_u = *(Nodes[u_id]);
    const Node& node_v = *(Nodes[v_id]);
    auto to_rad = [](double degree) { return degree * M_PI / 180.0; };
    
    double lat1 = to_rad(node_u.lat);
    double lon1 = to_rad(node_u.lon);
    double lat2 = to_rad(node_v.lat);
    double lon2 = to_rad(node_v.lon);
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
    std::cos(lat1) * std::cos(lat2) *
    std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
    return R * c;
}

   std::vector<int> Graph::handleShortesPath(SRTP sp, bool &possible, double &mtbd) {
    int n = static_cast<int>(Nodes.size());
    possible = false;

    // 1. Basic Bounds Check + forbidden endpoints
    if (sp.source < 0 || sp.source >= n || sp.target < 0 || sp.target >= n) {
        mtbd = -1;
        return {};
    }
    if (sp.forbidden_nodes.count(sp.source) || sp.forbidden_nodes.count(sp.target)) {
        possible = false;
        mtbd = -1;
        return {};
    }

    // 2. Priority Queue Setup (Min-Heap)
    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> min_heap;

    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    std::vector<uint8_t> visited(n, 0);

    // 3. Initialization
    dist[sp.source] = 0.0;
    min_heap.push({0.0, sp.source});

    // 4. Dijkstra Loop
    while (!min_heap.empty()) {
        auto [d, u] = min_heap.top(); 
        min_heap.pop();

        if (d > dist[u]) continue;
        if (visited[u]) continue;
        visited[u] = 1;

        if (u == sp.target) break; // Early exit - we popped the best distance for target

        for (auto *e : adj[u]) {
            if (!e) continue;

            // Defensive neighbor calculation:
            // If adj[u] stores outgoing edges, then e->u == u and neighbor v = e->v.
            // If adj[u] contains both directions (or you used same Edge for both), fall back to the other end.
            int v = (e->u == u) ? e->v : e->u;

            // Skip invalid neighbor indices
            if (v < 0 || v >= n) continue;

            // Constraint: Forbidden Road Types (map<int,bool> style)
            if (!sp.forbidden_road_types.empty()) {
                auto it_rt = sp.forbidden_road_types.find(e->road_type);
                if (it_rt != sp.forbidden_road_types.end() && it_rt->second) continue;
            }

            // Constraint: Forbidden Nodes (v)
            if (sp.forbidden_nodes.count(v)) continue;

            // Constraint: Forbidden Edges - check orientation (tail -> head = u -> v)
            if (!sp.forbidden_edges.empty() && sp.forbidden_edges.count({u, v})) continue;

            double length = e->length; // Mode fixed to distance

            // Relaxation
            double nd = dist[u] + length;
            if (nd < dist[v]) {
                dist[v] = nd;
                parent[v] = u;
                min_heap.push({dist[v], v});
            }
        }
    }

    // 5. Path Reconstruction
    std::vector<int> path;
    if (!std::isfinite(dist[sp.target])) {
        possible = false;
        mtbd = -1;
        return path;
    }

    possible = true;
    mtbd = dist[sp.target];

    for (int cur = sp.target; cur != -1; cur = parent[cur]) {
        path.push_back(cur);
    }
    std::reverse(path.begin(), path.end());
    
    return path;
}
