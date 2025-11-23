#include "kdt.h"
#include "graph.hpp"
#include <algorithm>
#include <queue>
#include <utility>
#include <cmath>

kdtree::kdtree() : root(nullptr) {}

kdtree::~kdtree() { clear(root); }

kdnode* kdtree::buildRecursive(std::vector<kdnode*>& points, int depth){
    if(points.empty()) return nullptr;
    int median = points.size()/2;
    if(depth%2 == 0){
        std::sort(points.begin(), points.end(), [](const kdnode* a, const kdnode* b) {
            return a->lat < b->lat; 
        });
    }
    else{
        std::sort(points.begin(), points.end(), [](const kdnode* a, const kdnode* b) {
            return a->lon < b->lon;
        });
    }
    kdnode* root = points[median];
    std::vector<kdnode*> left(points.begin(), points.begin() + median);
    std::vector<kdnode*> right(points.begin() + median + 1, points.end());
    root->left = buildRecursive(left,depth+1);
    root->right = buildRecursive(right,depth+1);
    return root;
}

kdnode* kdtree::insertRecursive(kdnode* root, kdnode* node, int depth){
    if (root == nullptr) {
        return node;
    }
    bool splitByLat = (depth % 2 == 0);

    if (splitByLat) {
        if (node->lat < root->lat) {
            root->left = insertRecursive(root->left, node, depth + 1);
        } else {
            root->right = insertRecursive(root->right, node, depth + 1);
        }
    } else {
        if (node->lon < root->lon) {
            root->left = insertRecursive(root->left, node, depth + 1);
        } else {

            root->right = insertRecursive(root->right, node, depth + 1);
        }
    }
    return root;
}

kdnode* kdtree::buildtree(Graph& graph){
    std::vector<kdnode*> kdNodes;
    for (auto node : graph.getNodes()) {
        if (node == nullptr) continue;

        kdnode* k = new kdnode(node->id,node->lat,node->lon,node->poi);
        kdNodes.push_back(k);
    }
    root = buildRecursive(kdNodes,0);
    return root;
}

void kdtree::add(const kdnode& node){
    kdnode* newNode = new kdnode(node);
    root = insertRecursive(root, newNode, 0);
}

bool kdtree::empty(){
    return (root == nullptr); 
}

kdnode* kdtree::getRoot(){
    return root;
}

void kdtree::clear(kdnode* node){
    if(node == nullptr) return;
    clear(node->left);
    clear(node->right);
    delete(node);
}


double distanceSquared(const kdnode* a, const kdnode* b) {
    double dlat = a->lat - b->lat;
    double dlon = a->lon - b->lon;
    return dlat * dlat + dlon * dlon;
}


void knnSearch(const kdnode* root, const kdnode& target, int depth, int k,
                        const std::string& target_poi,
                      std::priority_queue<std::pair<double, const kdnode*>>& maxHeap) {
    if (!root) return;
    double dist = distanceSquared(root, &target);

    if (target_poi.empty() || root->poi.count(target_poi)) { 
        maxHeap.push({dist, root});
        if ((int)maxHeap.size() > k) {
            maxHeap.pop(); 
        }
    }

    bool splitByLat = (depth % 2 == 0);
    double diff = splitByLat ? target.lat - root->lat : target.lon - root->lon;

    const kdnode* next = (diff < 0) ? root->left : root->right;
    const kdnode* other = (diff < 0) ? root->right : root->left;

    knnSearch(next, target, depth + 1, k,target_poi, maxHeap);
    double currentWorstDist = (maxHeap.empty()) ? INFINITY : maxHeap.top().first;

    if ((int)maxHeap.size() < k || diff * diff < currentWorstDist)
        knnSearch(other, target, depth + 1, k, target_poi, maxHeap);
}


std::vector<kdnode*> kdtree::knn(const kdnode* root, const kdnode& target, int k,const std::string& target_poi) {
    std::priority_queue<std::pair<double, const kdnode*>> maxHeap;
    knnSearch(root, target, 0, k, target_poi, maxHeap);

    std::vector<kdnode*> result;
    while (!maxHeap.empty()) {
        result.push_back(const_cast<kdnode*>(maxHeap.top().second));
        maxHeap.pop();
    }

    std::reverse(result.begin(), result.end());
    return result;
}


