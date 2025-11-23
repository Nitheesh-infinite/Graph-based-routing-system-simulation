#include "graph.hpp"
#include <algorithm>
#include <set>
#include <queue>
#include <limits>

// Helper to switch between distance and time based on preference
// You can hardcode this or pass a 'mode' string if KSP_E supports it.
static double get_weight(const Edge* e) {
    return e->length; // Defaulting to distance based on your prompt
}

// --------------------------------------------------------------------------
// Helper: Constrained Dijkstra
// Finds shortest path while ignoring specific nodes and edges.
// --------------------------------------------------------------------------
static std::pair<std::vector<int>, double> dijkstra_constrained(
    int src, 
    int tgt, 
    int N, 
    const std::vector<std::vector<Edge*>>& adj,
    const std::set<int>& banned_edges,  // IDs of edges to skip
    const std::set<int>& banned_nodes   // IDs of nodes to skip
) {
    std::priority_queue<std::pair<double, int>, 
                        std::vector<std::pair<double, int>>, 
                        std::greater<std::pair<double, int>>> pq;
    
    std::vector<double> dist(N, std::numeric_limits<double>::max());
    std::vector<int> parent(N, -1);

    dist[src] = 0;
    pq.push({0, src});

    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > dist[u]) continue;
        if (u == tgt) break;

        // Iterate adjacency list
        for (Edge* e : adj[u]) {
            // FIX: Determine the correct neighbor. 
            // If current node 'u' is the edge's start, neighbor is 'v'.
            // If current node 'u' is the edge's end, neighbor is 'u' (the other end).
            int v_node = (e->u == u) ? e->v : e->u;

            // Constraint 1: Skip Banned Edges
            if (banned_edges.count(e->id)) continue;

            // Constraint 2: Skip Banned Nodes
            if (banned_nodes.count(v_node)) continue;

            double w = get_weight(e); // Ensure this helper is correct
            
            if (dist[u] + w < dist[v_node]) {
                dist[v_node] = dist[u] + w;
                parent[v_node] = u;
                pq.push({dist[v_node], v_node});
            }
        }
    }

    // Reconstruct path
    if (dist[tgt] == std::numeric_limits<double>::max()) {
        return {{}, -1.0}; // No path found
    }

    std::vector<int> path;
    int curr = tgt;
    while (curr != -1) {
        path.push_back(curr);
        curr = parent[curr];
    }
    std::reverse(path.begin(), path.end());
    
    return {path, dist[tgt]};
}

// --------------------------------------------------------------------------
// Main: Yen's Algorithm Implementation
// --------------------------------------------------------------------------
std::vector<std::pair<std::vector<int>, double>> Graph::k_shortest_paths_exact(KSP_E ksp) {
    int source = ksp.source;
    int target = ksp.target;
    int K = ksp.k;

    // A holds the k shortest paths found so far [path, cost]
    std::vector<std::pair<std::vector<int>, double>> A;
    
    // B holds potential candidates. 
    // std::set automatically keeps them sorted by cost (first element of pair)
    // Format: {cost, path}
    std::set<std::pair<double, std::vector<int>>> B;

    // --- 1. Find the 1st shortest path ---
    auto first_path = dijkstra_constrained(source, target, N, adj, {}, {});
    
    if (first_path.first.empty()) {
        return A; // Source and target are disconnected
    }
    
    A.push_back(first_path);

    // --- 2. Find k-1 deviations ---
    for (int k = 1; k < K; ++k) {
        // The previous shortest path used to generate spurs
        const std::vector<int>& prev_path = A[k-1].first;

        // Iterate over the path nodes (except the last one)
        for (size_t i = 0; i < prev_path.size() - 1; ++i) {
            
            int spur_node = prev_path[i];
            
            // Root path is the sub-path from source to spur_node
            std::vector<int> root_path(prev_path.begin(), prev_path.begin() + i + 1);
            
            // Calculate cost of root path
           double root_path_cost = 0;
            for (size_t r = 0; r < root_path.size() - 1; ++r) {
                int u = root_path[r];
                int v = root_path[r+1];
                
                // FIX: Find edge connecting u and v (bi-directional check)
                for (Edge* e : adj[u]) {
                    int neighbor = (e->u == u) ? e->v : e->u; // <--- ADD THIS
                    if (neighbor == v) {                      // <--- CHECK NEIGHBOR, NOT e->v
                        root_path_cost += get_weight(e);
                        break;
                    }
                }
            }

            // Setup constraints
            std::set<int> banned_edges;
            std::set<int> banned_nodes;

            // a) Ban edges used in existing paths (A) that share the same root
            for (const auto& p_pair : A) {
                const std::vector<int>& p = p_pair.first;
                // If p shares the same root path
                if (p.size() > i + 1 && 
                    std::equal(root_path.begin(), root_path.end(), p.begin())) {
                    
                    int u = p[i];
                    int v = p[i+1];
                    
                    // Find and ban the specific edge ID
                    for (Edge* e : adj[u]) {
                        int neighbor = (e->u == u) ? e->v : e->u; // <--- ADD THIS
                        if (neighbor == v) {                      // <--- CHECK NEIGHBOR, NOT e->v
                            banned_edges.insert(e->id);
                            break; 
                        }
                    }
                }
            }

            // b) Ban nodes in root path to ensure loopless whole paths
            for (int node : root_path) {
                if (node != spur_node) {
                    banned_nodes.insert(node);
                }
            }

            // Calculate Spur Path
            auto spur_res = dijkstra_constrained(spur_node, target, N, adj, banned_edges, banned_nodes);
            
            if (!spur_res.first.empty()) {
                // Total Path = Root Path + Spur Path (exclude spur_node from spur path to avoid duplicate)
                std::vector<int> total_path = root_path;
                total_path.insert(total_path.end(), spur_res.first.begin() + 1, spur_res.first.end());
                
                double total_cost = root_path_cost + spur_res.second;
                
                B.insert({total_cost, total_path});
            }
        }

        if (B.empty()) break;

        // Move best candidate from B to A
        auto best_candidate = *B.begin();
        B.erase(B.begin()); // Remove from set

        // The set is {cost, path}, A expects {path, cost}
        A.push_back({best_candidate.second, best_candidate.first});
    }

    return A;
}