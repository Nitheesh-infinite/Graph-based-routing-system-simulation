#pragma once
#include "graph.hpp" 
#include <vector>
#include <tuple>
#include <map>
#include <string>

// Input Structure
struct fleet_all {
    // Tuple: <Order ID, Pickup Node, Drop Node>
    std::vector<std::tuple<int,int,int>> orders; 
    int num_delivery_guys;
    int depot_node;
    std::string objective = "sum_completion"; 
};

// Internal Order representation
struct Order {
    int id;
    int pickup_node;
    int delivery_node;
};

// Courier state
struct Courier_guy {
    int id;
    int current_location;
    std::vector<int> stops; // The sequence of Pickups/Drops/Depot
    std::vector<int> assigned_order_ids; // Track IDs
    double total_travel_time = 0.0;
};

// Output Structure
struct RouteInfo {
    int driver_id;
    std::vector<int> route; // Full node path
    std::vector<int> order_ids;
    double total_time;
};

struct FleetOutput {
    std::vector<RouteInfo> assignments;
    double total_delivery_time_s;
};

class FleetRouter {
private:
    Graph* graph;
    std::vector<Order> internal_orders;
    int num_couriers;
    int depot_location;

    // Cache: Key: {u, v}, Value: time
    std::map<std::pair<int,int>, double> dist_cache;

    // Helper: Get travel time with caching (Cost only)
    double get_travel_time(int u, int v);
    
    // Helper: Get full path between nodes (for final output)
    std::vector<int> get_path(int u, int v);

public:
    FleetRouter(Graph* g) : graph(g) {}

    FleetOutput optimize_all(fleet_all k);
    FleetOutput handle_dynamic_event(fleet_all k);
};