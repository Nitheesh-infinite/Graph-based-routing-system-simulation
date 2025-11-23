#include "graph.hpp"
#include "fleet.hpp" 
#include <fstream>
#include <algorithm>

Graph::Graph(const json& graph_json){
    if(graph_json.contains("nodes") && graph_json["nodes"].is_array()){
        for(auto& nd : graph_json["nodes"]){
            Node* node = new Node(nd["id"],nd["lat"],nd["lon"],nd["pois"]);
            Nodes.push_back(node);
        }
    }
    N = Nodes.size();
    adj.resize(N);
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

            Edge* edge = new Edge();
            edge->id = id; edge->u = u; edge->v = v;
            edge->length = length; edge->avg_time = avg_time;
            edge->oneway = oneway; edge->road_type = road_type;

            if (ed.contains("speed_profile")) {
                 for (const auto &s : ed["speed_profile"]) edge->speed_profile.push_back(s);
            }
            if(edge->speed_profile.empty()) edge->speed_profile.assign(1, avg_time > 0 ? length/avg_time : 10.0);

            Edges.push_back(edge);
            if (u >= 0 && u < N) adj[u].push_back(edge);
            if (!oneway && v >= 0 && v < N) adj[v].push_back(edge);
            if (v >= 0 && v < N) adj_r[v].push_back(edge);
            if (!oneway && u >= 0 && u < N) adj_r[u].push_back(edge);
        }
    }

};

json Graph::process_query(const json& query) {
    json res;
    // Note: The user input example doesn't strictly show a "type" field in the query object,
    // but typically in these drivers there is one. We will assume the structure
    // matches the "events" array loop in SampleDriver.
    // If query has "orders" and "fleet", treat as fleet problem.
    
    if (query.contains("orders") && query.contains("fleet")) {
        fleet_all input;
        
        // Parse Fleet (Handle the specific typo from prompt if present)
        if (query["fleet"].contains("num_delievery_guys")) {
            input.num_delivery_guys = query["fleet"]["num_delievery_guys"];
        } else if (query["fleet"].contains("num_delivery_guys")) {
            input.num_delivery_guys = query["fleet"]["num_delivery_guys"];
        } else {
            input.num_delivery_guys = 1;
        }
        
        input.depot_node = query["fleet"].value("depot_node", 0);
        
        // Parse Orders
        for(const auto& order_j : query["orders"]){
            int oid = order_j.value("order_id", -1); // Keys from prompt
            int pick = order_j.value("pickup", -1);
            int drop = order_j.value("dropoff", -1);
            input.orders.emplace_back(oid, pick, drop);
        }

        FleetRouter fr(this);
        FleetOutput fo = fr.optimize_all(input);

        // Construct specified Output Format
        res["assignments"] = json::array();
        
        for(const auto& r : fo.assignments) {
            json r_json;
            r_json["driver_id"] = r.driver_id;
            r_json["route"] = r.route;
            r_json["order_ids"] = r.order_ids;
            res["assignments"].push_back(r_json);
        }
        
        res["metrics"] = {
            {"total_delivery_time_s", fo.total_delivery_time_s}
        };
    } 
    else {
        // Fallback or Error
        res["error"] = "Unknown query format";
    }
    return res;
}