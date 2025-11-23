#include "../common/nlohmann/json.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include "graph.hpp"

using json = nlohmann::json;

int main(int argc, char* argv[]) {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    if (argc != 4) return 1;

    std::ifstream graph_file(argv[1]);
    json graph_json;
    graph_file >> graph_json;
    graph_file.close();

    Graph graph(graph_json);

    std::ifstream queries_file(argv[2]);
    json queries_input;
    queries_file >> queries_input;
    queries_file.close();

    json output_json;
    if (queries_input.contains("meta")) output_json["meta"] = queries_input["meta"];
    output_json["results"] = json::array();

    auto start_total = std::chrono::high_resolution_clock::now();

    if (queries_input.contains("events") && queries_input["events"].is_array()) {
        for (const auto& query : queries_input["events"]) {
            auto start_time = std::chrono::high_resolution_clock::now();
            json result;
            try {
                result = graph.process_query(query);
            } catch (...) {
                result = {{"id", query.value("id", -1)}, {"error", "exception"}, {"status", "failed"}};
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            result["processing_time"] = duration_ms;
            output_json["results"].push_back(result);
        }
    }
    
    auto end_total = std::chrono::high_resolution_clock::now();
    double dur = std::chrono::duration<double, std::milli>(end_total - start_total).count();
    std::cout << dur << std::endl;

    std::ofstream output_file(argv[3]);
    output_file << output_json.dump(4);
    output_file.close();

    return 0;
}