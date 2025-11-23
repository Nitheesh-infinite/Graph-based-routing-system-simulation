#include "graph.hpp"
#include <queue>
#include <algorithm>
#include <random>
#include <limits>
#include <chrono>

#define INF 1000000000.0

void Graph::run_dijkstra_core(int src, const std::vector<std::vector<Edge*>>& adjacency, std::vector<double>& dists) {
    std::fill(dists.begin(), dists.end(), INF);
    dists[src] = 0;

    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dists[u]) continue;

        for (auto* e : adjacency[u]) {
            int v = (e->u == u) ? e->v : e->u;
            if (dists[u] + e->length < dists[v]) {
                dists[v] = dists[u] + e->length;
                pq.push({dists[v], v});
            }
        }
    }
}

void Graph::precompute_alt() {
    if (alt_initialized) return;

    landmarks.clear();
    dist_from_landmark.assign(NUM_LANDMARKS, std::vector<double>(N, INF));
    dist_to_landmark.assign(NUM_LANDMARKS, std::vector<double>(N, INF));

    int start_node = 0;
    int max_degree = -1;
    for (int i = 0; i < N; ++i) {
        if ((int)adj[i].size() > max_degree) {
            max_degree = adj[i].size();
            start_node = i;
        }
    }
    landmarks.push_back(start_node);

    run_dijkstra_core(landmarks[0], adj, dist_from_landmark[0]);

    std::mt19937 rng(42);

    for (int k = 1; k < NUM_LANDMARKS; ++k) {
        int best_node = -1;
        double max_separation = -1.0;

        for (int u = 0; u < N; ++u) {
            double min_dist_to_any_L = INF;
            for (int i = 0; i < k; ++i) {
                min_dist_to_any_L = std::min(min_dist_to_any_L, dist_from_landmark[i][u]);
            }

            if (min_dist_to_any_L != INF && min_dist_to_any_L > max_separation) {
                max_separation = min_dist_to_any_L;
                best_node = u;
            }
        }

        if (best_node == -1) {
            std::uniform_int_distribution<int> uni(0, N - 1);
            best_node = uni(rng);
        }

        landmarks.push_back(best_node);
        run_dijkstra_core(best_node, adj, dist_from_landmark[k]);
    }

    for (int k = 0; k < NUM_LANDMARKS; ++k) {
        run_dijkstra_core(landmarks[k], adj_r, dist_to_landmark[k]);
    }

    alt_initialized = true;
}

double Graph::get_alt_heuristic(int u, int target) {
    if (!alt_initialized) return 0.0;

    double max_h = 0.0;

    for (int k = 0; k < NUM_LANDMARKS; ++k) {
        double d_u_to_L = dist_to_landmark[k][u];
        double d_t_to_L = dist_to_landmark[k][target];

        if (d_u_to_L < INF && d_t_to_L < INF) {
            max_h = std::max(max_h, d_u_to_L - d_t_to_L);
        }

        double d_L_to_u = dist_from_landmark[k][u];
        double d_L_to_t = dist_from_landmark[k][target];

        if (d_L_to_u < INF && d_L_to_t < INF) {
            max_h = std::max(max_h, d_L_to_t - d_L_to_u);
        }
    }

    return max_h;
}

double Graph::run_dijkstra_fallback(int source, int target) {
    std::vector<double> dist(N, INF);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    dist[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[u]) continue;
        if (u == target) return d;

        for (auto* edge : adj[u]) {
            int v = edge->v;
            if (dist[u] + edge->length < dist[v]) {
                dist[v] = dist[u] + edge->length;
                pq.push({dist[v], v});
            }
        }
    }
    return -1.0;
}

double Graph::weighted_a_star(int source, int target, double epsilon, double timeout) {
    double w = 1.0 + (epsilon / 100.0);
    double mu = INF;

    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq_f, pq_b;
    std::vector<double> g_f(N, INF), g_b(N, INF);
    std::vector<bool> visited_f(N, false), visited_b(N, false);

    g_f[source] = 0.0;
    pq_f.push({w * get_alt_heuristic(source, target), source});

    g_b[target] = 0.0;
    pq_b.push({w * get_alt_heuristic(source, target), target});

    auto start_time = std::chrono::high_resolution_clock::now();

    while (!pq_f.empty() && !pq_b.empty()) {
        if (timeout > 0) {
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - start_time;
            if (elapsed.count() > timeout && mu < INF) return mu;
        }

        if (pq_f.top().first + pq_b.top().first >= mu) {
            return mu;
        }

        if (pq_f.size() <= pq_b.size()) {
            auto [f, u] = pq_f.top(); pq_f.pop();

            if (f > g_f[u] + w * get_alt_heuristic(u, target)) continue;
            visited_f[u] = true;

            if (visited_b[u]) {
                mu = std::min(mu, g_f[u] + g_b[u]);
            }

            for (auto* edge : adj[u]) {
                int v = edge->v;
                double weight = edge->length;

                if (g_f[u] + weight < g_f[v]) {
                    g_f[v] = g_f[u] + weight;
                    pq_f.push({g_f[v] + w * get_alt_heuristic(v, target), v});
                    
                    if (visited_b[v]) {
                        mu = std::min(mu, g_f[v] + g_b[v]);
                    }
                }
            }
        } else {
            auto [f, u] = pq_b.top(); pq_b.pop();

            if (f > g_b[u] + w * get_alt_heuristic(source, u)) continue;
            visited_b[u] = true;

            if (visited_f[u]) {
                mu = std::min(mu, g_f[u] + g_b[u]);
            }

            for (auto* edge : adj_r[u]) {
                int v = (edge->v == u) ? edge->u : edge->v;
                double weight = edge->length;

                if (g_b[u] + weight < g_b[v]) {
                    g_b[v] = g_b[u] + weight;
                    pq_b.push({g_b[v] + w * get_alt_heuristic(source, v), v});

                    if (visited_f[v]) {
                        mu = std::min(mu, g_f[v] + g_b[v]);
                    }
                }
            }
        }
    }

    if (mu < INF) return mu;

    return run_dijkstra_fallback(source, target);
}