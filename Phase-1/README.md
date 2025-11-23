# Dynamic Graph Algorithms with Periodic Updates and Queries

This document presents implementations and explanations of standard algorithms for dynamic graphs that support periodic updates and queries, focusing on shortest path calculations and K-Nearest Neighbors (KNN).

## Queries

### Shortest Path

- **Minimizing Distance**  
  Use classic shortest path algorithms like Dijkstra’s algorithm to find the shortest path by minimizing edge distance.

- **Minimizing Time with Speed Limits as a Function of Time (Changing Traffic)**  
  Implement time-dependent shortest path algorithms where edge weights vary with departure time, modeling traffic changes dynamically.

- **Constraints on Edges/Nodes**  
  Handle forbidden roads or restricted nodes by excluding them from the search or treating them as edges with infinite weight during shortest path computations.

### KNN (K Nearest Neighbors)

- **Based on Euclidean Distance**  
  Compute Euclidean distances between node coordinates to find the nearest neighbors.

- **Based on Shortest Path Distance**  
  Use shortest path distances within the graph as the metric to determine the K nearest neighbors.

---
