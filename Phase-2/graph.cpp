#include"graph.hpp"
#include<fstream>
#include<algorithm>

Graph::Graph(const json& graph_json){
    if(graph_json.contains("nodes") && graph_json["nodes"].is_array()){
        for(auto& nd : graph_json["nodes"]){
            Node* node = new Node(nd["id"],nd["lat"],nd["lon"],nd["pois"]);
            Nodes.push_back(node);
            
        }
    }
    current_token = 0;
    N = Nodes.size();
    adj.clear();
    adj.resize(N);
    adj_r.clear();
    adj_r.resize(N);
    if (graph_json.contains("edges") && graph_json["edges"].is_array()) {
        for (const auto &ed : graph_json["edges"]) {
            int id = ed.value("id", -1);
            int u  = ed.value("u", -1);
            int v  = ed.value("v", -1);
            double length = ed.value("length", 0.0);
            double avg_time = ed.value("avg_time", 0.0);
            bool oneway = ed.value("oneway", true);
            std::string road_type = ed.value("road_type", std::string());

            // create Edge*
            Edge* edge = new Edge();
            edge->id = id;
            edge->u = u;
            edge->v = v;
            edge->length = length;
            edge->avg_time = avg_time;
            edge->oneway = oneway;
            edge->road_type = road_type;

            // speed_profile: copy if present, otherwise fill with default 96 slots
            edge->speed_profile.clear();
            if (ed.contains("speed_profile") && ed["speed_profile"].is_array()) {
                for (const auto &s : ed["speed_profile"]) {
                    edge->speed_profile.push_back(s.get<double>());
                }
            }
            if (edge->speed_profile.empty()) {
                edge->speed_profile.assign(96, (edge->length/edge->avg_time));
            }
            Edges.push_back(edge);
            // bounds check for u/v
            if (u >= 0 && u < N) adj[u].push_back(edge);
            if (!oneway && v >= 0 && v < N) adj[v].push_back(edge);
            // keep reverse adjacency if you need it elsewhere
            if (v >= 0 && v < N) adj_r[v].push_back(edge);
            if (!oneway && u >= 0 && u < N) adj_r[u].push_back(edge);
        }
    }
   
    
    edge_visited_token.assign(Edges.size()+1,0);
     edge_lookup.clear();
    if (adj.empty()) return;
    for (size_t u = 0; u < adj.size(); ++u) {
        for (Edge* e : adj[u]) {
            if (!e) continue;
            std::pair<int,int> key_forward = {e->u, e->v};
            // insert only if key not present to keep the first-seen edge
            edge_lookup.emplace(key_forward, e);
            // if edge is bidirectional, also map reverse pair
            if (!e->oneway) {
                std::pair<int,int> key_rev = {e->v, e->u};
                edge_lookup.emplace(key_rev, e);
            }
        }
    }
    buildSCC();
    precompute_alt();
};
void Graph::dfs_fill_order(int u, std::vector<bool>& visited, std::vector<int>& stack) {
    visited[u] = true;
    for (Edge* e : adj[u]) {
       
        int v = (e->u == u) ? e->v : e->u;
        if (!visited[v]) {
            dfs_fill_order(v, visited, stack);
        }
    }
    stack.push_back(u);
}

// Pass 2: Traverse reversed graph to assign components
void Graph::dfs_assign_component(int u, int c_id, std::vector<bool>& visited) {
    visited[u] = true;
    component_id[u] = c_id;
    for (Edge* e : adj_r[u]) {
        
        int v = (e->v == u) ? e->u : e->v;
        if (!visited[v]) {
            dfs_assign_component(v, c_id, visited);
        }
    }
}

void Graph::buildSCC() {
    component_id.assign(N, -1);
    scc_count = 0;
    std::vector<bool> visited(N, false);
    std::vector<int> stack;

    // 1. Fill stack (Forward pass)
    for (int i = 0; i < N; i++) {
        if (!visited[i]) {
            dfs_fill_order(i, visited, stack);
        }
    }

    // 2. Process in reverse order (Reverse pass)
    std::fill(visited.begin(), visited.end(), false);
    while (!stack.empty()) {
        int u = stack.back();
        stack.pop_back();
        if (!visited[u]) {
            dfs_assign_component(u, scc_count, visited);
            scc_count++;
        }
    }
}

bool Graph::hasPath(int u, int v) {
    if (u < 0 || u >= N || v < 0 || v >= N) return false;
    if (u == v) return true;

    
    if (component_id[u] != -1 && component_id[u] == component_id[v]) {
        return true;
    }


    std::queue<int> q;
    q.push(u);
    std::vector<bool> visited(N, false);
    visited[u] = true;

    while(!q.empty()){
        int curr = q.front();
        q.pop();

        if(curr == v) return true;

       
        if (component_id[curr] == component_id[v] && component_id[v] != -1) return true;

        for(auto e : adj[curr]){
            int neighbor = (e->u == curr) ? e->v : e->u;
            if(!visited[neighbor]){
                visited[neighbor] = true;
                q.push(neighbor);
            }
        }
    }
    return false;
   }


json Graph::process_query(const json& query) {
    json res;
    if (!query.contains("type")){
        res["error"] = "missing type";
        return res;
    }
    res["id"] = query["id"];
    std::string t = query["type"];
    if (t == "k_shortest_paths"){
        KSP_E sp;
        sp.source = query["source"];
        sp.target = query["target"];
        sp.k = query["k"];
        // if(query.contains("mode")) sp.mode = query["mode"];
        // if (query.contains("constraints")){
        // if (query["constraints"].contains("forbidden_nodes")) {
        //     for (const auto& fn : query["constraints"]["forbidden_nodes"]) sp.forbidden_nodes[fn] = true;
        // }
        // if (query["constraints"].contains("forbidden_road_types")) {
        //     if (query["constraints"].contains("forbidden_road_types") && query["constraints"]["forbidden_road_types"].is_array()) {
        //     for (const auto& val : query["constraints"]["forbidden_road_types"]) {
        //       sp.forbidden_road_types[val] = true;
        //             }
        //         }   
        //     }
        //}
        std::vector<std::pair<std::vector<int>, double>> paths = k_shortest_paths_exact(sp);
        res["paths"] = json::array();
        for (const auto& entry : paths){
            json j_entry;
            j_entry["path"] = entry.first;
            j_entry["length"] = entry.second;
            res["paths"].push_back(j_entry);
        }
    }else if(t == "k_shortest_paths_heuristic"){
        KSP_H ksp;
        ksp.id = query["id"];
        ksp.source = query["source"];
        ksp.target = query["target"];
        ksp.k = query["k"];
        ksp.overlap_threshold = query["overlap_threshold"];
        std::vector<std::pair<std::vector<int>,double>> paths = k_shortest_paths_heustirics(ksp);
        res["paths"] = json::array();
        for (const auto& entry : paths){
            json j_entry;
            j_entry["path"] = entry.first;
            j_entry["length"] = entry.second;
            res["paths"].push_back(j_entry);
        }
    }else if(t == "approx_shortest_path"){
        double epsilon = query["acceptable_error_pct"];
        double timeout = query["time_budget_ms"];
        for(auto vals : query["queries"]){
            json j_entry;
            j_entry["source"] = vals["source"];
            j_entry["target"] = vals["target"];
            j_entry["approx_shortest_distance"] = weighted_a_star(vals["source"], vals["target"],epsilon,timeout); 
            res["distances"].push_back(j_entry);
        }
    }else {
            res["error"] = "unsupported query type: " + t;
    }
    return res;
}
