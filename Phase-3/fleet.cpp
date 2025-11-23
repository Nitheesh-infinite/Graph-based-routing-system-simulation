#include "fleet.hpp"
#include <limits>
#include <algorithm>
#include <iostream>

const double INF = std::numeric_limits<double>::infinity();
const double UNREACHABLE_PENALTY = 1e9;

double FleetRouter::get_travel_time(int u, int v) {
    if (u == v) return 0.0;
    
    std::pair<int,int> key = {u, v};
    if (dist_cache.count(key)) return dist_cache[key];

    SRTP sp;
    sp.source = u;
    sp.target = v;
    bool possible = false;
    double time = 0.0;
    
    // Uses the Optimized A* from srtp.cpp (only calculates cost/time)
    // We don't strictly need the path vector here, just the cost
    graph->A_star_time(sp, possible, time);
    
    if (!possible) time = UNREACHABLE_PENALTY;
    
    dist_cache[key] = time;
    return time;
}

// Helper to retrieve actual path nodes for final output
std::vector<int> FleetRouter::get_path(int u, int v) {
    if (u == v) return {};
    
    SRTP sp;
    sp.source = u;
    sp.target = v;
    bool possible = false;
    double time = 0.0;
    
    // Calls A* and returns the path vector
    return graph->A_star_time(sp, possible, time);
}

FleetOutput FleetRouter::optimize_all(fleet_all k) {
    this->num_couriers = k.num_delivery_guys;
    this->depot_location = k.depot_node;
    this->internal_orders.clear();
    
    for(const auto& t : k.orders) {
        Order o;
        o.id = std::get<0>(t);
        o.pickup_node = std::get<1>(t);
        o.delivery_node = std::get<2>(t);
        internal_orders.push_back(o);
    }

    // Initialize Couriers
    std::vector<Courier_guy> fleet(num_couriers);
    for (int i = 0; i < num_couriers; ++i) {
        fleet[i].id = i;
        fleet[i].stops.push_back(depot_location); 
        fleet[i].total_travel_time = 0.0;
    }

    std::vector<int> unassigned_indices;
    for(size_t i=0; i<internal_orders.size(); ++i) unassigned_indices.push_back(i);

    // Greedy Insertion
    while (!unassigned_indices.empty()) {
        int best_order_idx = -1;
        int best_courier = -1;
        int best_p_pos = -1; 
        int best_d_pos = -1; 
        double min_obj_increase = INF;

        for (int oid : unassigned_indices) {
            const Order& order = internal_orders[oid];
            
            for (int c = 0; c < num_couriers; ++c) {
                const auto& stops = fleet[c].stops;
                double current_val = fleet[c].total_travel_time;
                
                // i starts at 1 because 0 is fixed Depot
                for (size_t i = 1; i <= stops.size(); ++i) {
                    
                    int p_prev = stops[i-1];
                    int p_curr = (i < stops.size()) ? stops[i] : -1;
                    
                    double delta_p = get_travel_time(p_prev, order.pickup_node);
                    if (p_curr != -1) {
                        delta_p += get_travel_time(order.pickup_node, p_curr);
                        delta_p -= get_travel_time(p_prev, p_curr);
                    }

                    for (size_t j = i; j <= stops.size(); ++j) {
                        double delta_d = 0.0;
                        if (j == i) {
                            delta_d += get_travel_time(order.pickup_node, order.delivery_node);
                            if (p_curr != -1) {
                                delta_d += get_travel_time(order.delivery_node, p_curr);
                                delta_d -= get_travel_time(order.pickup_node, p_curr);
                            }
                        } else {
                            int d_prev = stops[j-1];
                            int d_curr = (j < stops.size()) ? stops[j] : -1;
                            delta_d += get_travel_time(d_prev, order.delivery_node);
                            if (d_curr != -1) {
                                delta_d += get_travel_time(order.delivery_node, d_curr);
                                delta_d -= get_travel_time(d_prev, d_curr);
                            }
                        }

                        double added = delta_p + delta_d;
                        double new_total = current_val + added;
                        
                        // Objective check (Defaulting to Sum Completion logic)
                        if (added < min_obj_increase) {
                            min_obj_increase = added;
                            best_order_idx = oid;
                            best_courier = c;
                            best_p_pos = i;
                            best_d_pos = j;
                        }
                    }
                }
            }
        }

        if (best_order_idx != -1) {
            auto& cr = fleet[best_courier];
            
            // Insert Pickup
            cr.stops.insert(cr.stops.begin() + best_p_pos, internal_orders[best_order_idx].pickup_node);
            // Insert Drop
            cr.stops.insert(cr.stops.begin() + best_d_pos + 1, internal_orders[best_order_idx].delivery_node);
            
            // Track Order ID
            cr.assigned_order_ids.push_back(internal_orders[best_order_idx].id);

            // Update Cost
            double t = 0;
            for(size_t k=0; k<cr.stops.size()-1; ++k) 
                t += get_travel_time(cr.stops[k], cr.stops[k+1]);
            cr.total_travel_time = t;

            unassigned_indices.erase(std::remove(unassigned_indices.begin(), unassigned_indices.end(), best_order_idx), unassigned_indices.end());
        } else {
            break; 
        }
    }

    // Populate Output
    FleetOutput output;
    output.total_delivery_time_s = 0;

    for (const auto& c : fleet) {
        RouteInfo ri;
        ri.driver_id = c.id;
        ri.order_ids = c.assigned_order_ids;
        ri.total_time = c.total_travel_time;
        
        // RECONSTRUCT FULL PATH
        // The 'stops' vector contains [Depot, P1, P2, D1...]
        // We need the full node-by-node path between these stops.
        if (!c.stops.empty()) {
            ri.route.push_back(c.stops[0]);
            for (size_t k = 0; k < c.stops.size() - 1; ++k) {
                int u = c.stops[k];
                int v = c.stops[k+1];
                // Get detailed path from u to v
                std::vector<int> segment = get_path(u, v);
                
                // segment contains [u, ..., v]
                // We append everything AFTER u to avoid duplication
                for (size_t m = 1; m < segment.size(); ++m) {
                    ri.route.push_back(segment[m]);
                }
            }
        } else {
             // If strictly empty, just depot
             ri.route.push_back(depot_location);
        }
        
        output.assignments.push_back(ri);
        output.total_delivery_time_s += c.total_travel_time;
    }

    return output;
}

FleetOutput FleetRouter::handle_dynamic_event(fleet_all k) {
    // Fallback to normal optimize for this phase
    return optimize_all(k);
}