#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

using namespace std;

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
 private:
  // TODO_STUDENT
  unordered_map<VertexT, unordered_map<VertexT, WeightT>> edges;
  size_t numEdge;
 public:
  /// Default constructor
  graph() {
    // TODO_STUDENT
    numEdge = 0;
    
  }

  /// @brief Add the vertex `v` to the graph, must typically be O(1).
  /// @param v
  /// @return true if successfully added; false if it existed already
  bool addVertex(VertexT v) {
    // TODO_STUDENT
    return edges.emplace(v, unordered_map<VertexT, WeightT>{}).second;
  }

  /// @brief Add or overwrite directed edge in the graph, must typically be
  /// O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight edge weight / label
  /// @return true if successfully added or overwritten;
  ///         false if either vertices isn't in graph
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    // TODO_STUDENT
    if (edges.count(from) == 0 || edges.count(to) == 0) {
      return false;
    }
    bool checkNewEdge = !edges[from].count(to); 
    edges[from][to] = weight;
    if (checkNewEdge==true) {
      ++numEdge;
    }
    return true;
  }

  /// @brief Maybe get the weight associated with a given edge, must typically
  /// be O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight output parameter
  /// @return true if the edge exists, and `weight` is set;
  ///         false if the edge does not exist
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    // TODO_STUDENT
    if (!edges.contains(from)){
      return false;
    } 
    if (!edges.at(from).contains(to)){
      return false;
    }
    weight = edges.at(from).at(to);
    return true;
  }

  /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
  /// @param v
  /// @return vertices that v has an edge to
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;
    if (edges.count(v)) {
      for (const auto& connection : edges.at(v)) {
        S.insert(connection.first);
      }
    }
    return S;
  }

  /// @brief Return a vector containing all vertices in the graph
  vector<VertexT> getVertices() const {
    // TODO_STUDENT
    vector<VertexT> result;
    for (const auto& pair : edges) {
      result.push_back(pair.first);
    }
    return result;
  }

  /// @brief Get the number of vertices in the graph. Runs in O(1).
  size_t numVertices() const {
    // TODO_STUDENT
    return edges.size();
  }

  /// @brief Get the number of directed edges in the graph. Runs in at most
  /// O(|V|), but should be O(1).
  size_t numEdges() const {
    // TODO_STUDENT
    return numEdge;
  }
};
