#include "application.h"
#include <iostream>
#include <limits>
#include <map>
#include <queue>          // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "dist.h"
#include "graph.h"
#include "json.hpp"       

using namespace std;
using json = nlohmann::json;

double INF = numeric_limits<double>::max();

void buildGraph(istream& input,
                graph<long long, double>& g,
                vector<BuildingInfo>& buildings,
                unordered_map<long long, Coordinates>& coords) {
  json data;
  input >> data;

  auto waypoints = data.value("waypoints", json::array());
  auto buildingsJ= data.value("buildings",  json::array());
  auto footways   = data.value("footways",   json::array());

  for (auto& wp : waypoints) {
    long long id  = wp["id"];
    double   lat = wp["lat"];
    double   lon = wp["lon"];
    coords[id] = { lat, lon };
    g.addVertex(id);
  }

  for (auto& b : buildingsJ) {
    long long id  = b["id"];
    double   lat = b["lat"];
    double   lon = b["lon"];
    g.addVertex(id);
  
    BuildingInfo bi;
    bi.id       = id;
    bi.location = { lat, lon };       
    bi.name     = b.value("name", "");
    bi.abbr     = b.value("abbr", "");
    buildings.push_back(bi);
  }

  for (auto& way : footways) {
    for (size_t i = 1; i < way.size(); ++i) {
      long long u = way[i - 1];
      long long v = way[i];
      if (!coords.count(u) || !coords.count(v)) continue;
      double d = distBetween2Points(coords[u], coords[v]);
      g.addEdge(u, v, d);
      g.addEdge(v, u, d);
    }
  }

  for (auto& bInfo : buildings) {
    for (auto& [id, pt] : coords) {
      if (id == bInfo.id) continue;
      double d = distBetween2Points(bInfo.location, pt);
      if (d <= 0.036) {
        g.addEdge(bInfo.id, id, d);
        g.addEdge(id, bInfo.id, d);
      }
    }
  }
}

BuildingInfo getBuildingInfo(const vector<BuildingInfo>& buildings,
                             const string& query) {
  for (const auto& building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo>& buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const auto& building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

vector<long long> dijkstra(const graph<long long, double>& G, long long start, long long target, const set<long long>& ignoreNodes) {
    unordered_map<long long, double> dist;
    unordered_map<long long, long long> prev;
    priority_queue<pair<double, long long>, vector<pair<double, long long>>, greater<> > fringe;
    vector<long long> path;
    if (start == target) {
        path.push_back(start);
        return path;
    }
    for (auto v : G.getVertices()) {
        dist[v] = INF;
    }
    dist[start] = 0.0;
    fringe.push({0.0, start});
    while (!fringe.empty()) {
      auto [d_u, u] = fringe.top();
      fringe.pop();
      if (d_u > dist[u] || u == target) {
          if (u == target) break;
          else continue;
      }
      for (auto nbr : G.neighbors(u)) {
        if (ignoreNodes.count(nbr) && nbr != start && nbr != target) 
            continue;
        double w;
        if (!G.getWeight(u, nbr, w)) continue;
        double alt = d_u + w;
        if (alt < dist[nbr]) {
            dist[nbr] = alt;
            prev[nbr] = u;
            fringe.push({alt, nbr});
          }
      }
    }
    if (dist[target] == INF) {
      return {};
    }
    long long cur = target;
    while (cur != start) {
        path.push_back(cur);
        cur = prev[cur];
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    return path;
}

double routeDist(const graph<long long,double>& G, const vector<long long>& path) {
  double length = 0.0, w;
  for (size_t i = 1, n = path.size(); i < n; ++i) {
      if (!G.getWeight(path[i-1], path[i], w)) return -1;
      length += w;
  }
  return length;
}

void printRoute(const vector<long long>& path) {
  for (size_t i = 0; i < path.size(); ++i) {
    cout << path[i];
    if (i + 1 < path.size()) cout << "->";
  }
  cout << "\n";
}

static void displayInfo(const string& title, const BuildingInfo& info) {
  cout << "\n" << title << ":\n"
       << " " << info.name  << "\n"
       << " " << info.id    << "\n"
       << " (" << info.location.lat << ", "
                << info.location.lon << ")\n";
}

void navigateUI(const vector<BuildingInfo>& allBuildings,
              const graph<long long,double>& graphData) {
  set<long long> blocked;
  for (auto const& bld : allBuildings) 
      blocked.insert(bld.id);
  string input1, input2;
  cout << "\nEnter person 1's building (partial name or abbreviation), or #> ";
  while (true) {
      if (!getline(cin, input1) || input1 == "#") 
          break;
      cout << "Enter person 2's building (partial name or abbreviation)> ";
      getline(cin, input2);
      auto personA = getBuildingInfo(allBuildings, input1);
      if (personA.id < 0) {
          cout << "Person 1's building not found\n\n"
               << "Enter person 1's building (partial name or abbreviation), or #> ";
          continue;
      }
      auto personB = getBuildingInfo(allBuildings, input2);
      if (personB.id < 0) {
          cout << "Person 2's building not found\n\n"
               << "Enter person 1's building (partial name or abbreviation), or #> ";
          continue;
      }
      displayInfo("Person 1's point", personA);
      displayInfo("Person 2's point", personB);
      auto midpoint = centerBetween2Points(personA.location, personB.location);
      auto destination = getClosestBuilding(allBuildings, midpoint);
      displayInfo("Destination Building", destination);
      auto routeA = dijkstra(graphData, personA.id, destination.id, blocked);
      auto routeB = dijkstra(graphData, personB.id, destination.id, blocked);
      if (routeA.empty() || routeB.empty()) {
          cout << "\nAt least one person was unable to reach the destination building. Is an edge missing?\n\n";
      } else {
          cout << "\nPerson 1's distance to dest: " 
               << routeDist(graphData, routeA) << " miles\nPath: ";
          printRoute(routeA);
          cout << "\nPerson 2's distance to dest: " 
               << routeDist(graphData, routeB) << " miles\nPath: ";
          printRoute(routeB);
      }
      cout << "\nEnter person 1's building (partial name or abbreviation), or #> ";
  }
}
