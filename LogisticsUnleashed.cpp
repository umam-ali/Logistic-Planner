#include <iostream>
#include <vector>
#include <climits>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <utility>
#include <algorithm>

using namespace std;

const int INF = 1e9;

int N, T, M, K, F;
vector<int> hubs, houses, fuel_stations;
vector<vector<int>> dist, next_node;
unordered_map<int, unordered_map<int, int>> edge_map;
unordered_set<int> fuel_station_set;

// Floyd-Warshall Algorithm
void floyd_warshall(const vector<vector<pair<int, int>>>& graph) {
    dist.assign(T, vector<int>(T, INF));
    next_node.assign(T, vector<int>(T, -1));
    
    for (int u = 0; u < T; ++u) {
        dist[u][u] = 0;
        next_node[u][u] = u;
    }
    
    for (int u = 0; u < T; ++u) {
        for (int i = 0; i < graph[u].size(); ++i) {
            int v = graph[u][i].first;
            int cost = graph[u][i].second;
            dist[u][v] = cost;
            next_node[u][v] = v;
        }
    }
    for (int k = 0; k < T; ++k) {
        for (int i = 0; i < T; ++i) {
            for (int j = 0; j < T; ++j) {
                if (dist[i][k] < INF && dist[k][j] < INF &&
                    dist[i][j] > dist[i][k] + dist[k][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    next_node[i][j] = next_node[i][k];
                }
            }
        }
    }
}

vector<int> get_path(int u, int v) {
    vector<int> path;
    if (next_node[u][v] == -1) return path;
    path.push_back(u);
    while (u != v) {
        u = next_node[u][v];
        path.push_back(u);
    }
    return path;
}

int farthest_from_fuel(const vector<int>& points) {
    int max_dist = -1;
    int selected = -1;
    for (int i = 0; i < points.size(); ++i) {
        int p = points[i];
        int min_to_station = INF;
        for (int j = 0; j < fuel_stations.size(); ++j) {
            int s = fuel_stations[j];
            if (dist[p][s] < min_to_station) {
                min_to_station = dist[p][s];
            }
        }
        if (min_to_station > max_dist) {
            max_dist = min_to_station;
            selected = p;
        }
    }
    return selected;
}

bool append_path_with_fuel(vector<int>& route, int from, int to, int& fuel) {
    vector<int> path = get_path(from, to);
    for (int i = 1; i < path.size(); ++i) {
        int u = path[i - 1];
        int v = path[i];
        if (edge_map[u].count(v) == 0) return false;
        int cost = edge_map[u][v];
        if (cost > fuel) return false;
        fuel -= cost;
        route.push_back(v);
        if (fuel_station_set.count(v)) fuel = F;
    }
    return true;
}

int main() {
    cin >> N >> T >> M >> K >> F;
    hubs.resize(N);
    houses.resize(N);
    fuel_stations.resize(K);

    for (int i = 0; i < N; ++i) cin >> hubs[i];
    for (int i = 0; i < N; ++i) cin >> houses[i];
    for (int i = 0; i < K; ++i) {
        cin >> fuel_stations[i];
        fuel_station_set.insert(fuel_stations[i]);
    }

    vector<vector<pair<int, int>>> graph(T);
    for (int i = 0; i < M; ++i) {
        int u, v, c;
        cin >> u >> v >> c;
        graph[u].push_back(make_pair(v, c));
        graph[v].push_back(make_pair(u, c));
        edge_map[u][v] = c;
        edge_map[v][u] = c;
    }

    floyd_warshall(graph);

    int start_hub = farthest_from_fuel(hubs);
    int end_house = farthest_from_fuel(houses);

    unordered_set<int> visited_hubs, visited_houses;
    unordered_map<int, int> hub_for_house;
    for (int i = 0; i < N; ++i) hub_for_house[houses[i]] = hubs[i];

    vector<int> route;
    int current = start_hub;
    int fuel = F;
    route.push_back(current);
    visited_hubs.insert(current);

    // Visit all hubs
    while (visited_hubs.size() < hubs.size()) {
        int next = -1, best_dist = INF;
        for (int i = 0; i < hubs.size(); ++i) {
            int h = hubs[i];
            if (visited_hubs.count(h)) continue;
            if (dist[current][h] < best_dist) {
                best_dist = dist[current][h];
                next = h;
            }
        }
        if (next == -1) break;
        if (!append_path_with_fuel(route, current, next, fuel)) {
            cout << "Failed fuel constraint while visiting hubs.\n";
            return 0;
        }
        visited_hubs.insert(next);
        current = next;
    }

    for (int i = 0; i < houses.size(); ++i) {
        int h = houses[i];
        if (h == end_house) continue;
        if (!append_path_with_fuel(route, current, h, fuel)) {
            cout << "Failed fuel constraint while visiting houses.\n";
            return 0;
        }
        visited_houses.insert(h);
        current = h;
    }

    // Visit final house
    if (!append_path_with_fuel(route, current, end_house, fuel)) {
        cout << "Failed fuel constraint on last house.\n";
        return 0;
    }
    visited_houses.insert(end_house);

    cout << route.size() << "\n";
    for (int i = 0; i < route.size(); ++i) {
        cout << route[i] << " ";
    }
    cout << "\n";

    return 0;
}