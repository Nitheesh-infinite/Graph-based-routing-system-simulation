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
    kd_tree = new kdtree();
    kd_tree->buildtree(*this);
};
json Graph::handleRemoveEdge(const json& query) {
    json res;
    int eid = query["edge_id"];
    Edge* target = nullptr;
    for (auto& p : Edges) {
            if (p->id == eid) { target = p; break; }
        }
    if (!target) {
        res["error"] = "edge not found";
        res["removed"] = false;
        return res;
    }
    if(target->active == false) res["removed"] = false;
    else res["removed"] = true;
    res["edge_id"] = eid;
    return res;
};

json Graph::handleModifyEdge(const json& query) {
    json res;
    if (query.contains("id")) {
        res["id"] = query["id"];
    }

    int eid = query["edge_id"];
    const json& patch = query["patch"];

    Edge* target = nullptr;
    for (auto& p : Edges) {
        if (p->id == eid) { target = p; break; }
    }

    if (!target) {
        res["done"] = false;
        return res;
    }

    if (target->active) {
        if (patch.empty()) {
            res["done"] = false;
            return res;
        }
    } else {
        target->active = true;
    }

    if (patch.contains("length")) target->length = patch["length"];
    if (patch.contains("average_time")) target->avg_time = patch["average_time"];
    if (patch.contains("speed_profile")) target->speed_profile = patch["speed_profile"].get<std::vector<double>>();
    if (patch.contains("road_type")) target->road_type = patch["road_type"];

    res["done"] = true;
    return res;
}

json Graph::process_query(const json& query) {
    json res;
    if (!query.contains("type")){
        res["error"] = "missing type";
        return res;
    }
    res["id"] = query["id"];
    std::string t = query["type"];
    if (t == "remove_edge"){
        res = handleRemoveEdge(query);
    }
    else if (t == "modify_edge"){
        res = handleModifyEdge(query);
    }
    else if (t == "shortest_path"){
        SRTP sp;
        sp.id = query["id"];
        sp.source = query["source"];
        sp.target = query["target"];
        sp.mode = query["mode"];
        if (query.contains("constraints")){
        if (query["constraints"].contains("forbidden_nodes")) {
            for (const auto& fn : query["constraints"]["forbidden_nodes"]) sp.forbidden_nodes[fn] = true;
        }
        if (query["constraints"].contains("forbidden_road_types")) {
            if (query["constraints"].contains("forbidden_road_types") && query["constraints"]["forbidden_road_types"].is_array()) {
            for (const auto& val : query["constraints"]["forbidden_road_types"]) {
              sp.forbidden_road_types[val] = true;
            }
            }
        }
    }

        double mtbd = 0.0;
        bool possible = false;
        std::vector<int> path ;
        if(query["mode"] == "distance") path = handleShortesPath(sp, possible, mtbd);
        else if(query["mode"] == "time") path = A_star_time(sp, possible, mtbd);
        else res["error"] = "mode not found";
        res["possible"] = possible;
        res["minimum_time/minimum_distance"] = mtbd;
        res["path"] = path;
    } else if (t == "knn"){
        KNN kn(query["id"],query["poi"],query["query_point"]["lat"],query["query_point"]["lon"],query["k"],query["metric"]);

        std::vector<int> ans = handleKnn(kn);
        res["result"] = ans;
    }
    else {
            res["error"] = "unsupported query type: " + t;
    }
    return res;
}
