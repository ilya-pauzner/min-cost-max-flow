#include <bits/stdc++.h>

using namespace std;

class Graph;

struct Edge {
    int from;
    int to;
    int weight;
    int flow;
    int capacity;
    size_t id;

    bool operator==(const Edge& other) const {
        return from == other.from and to == other.to and weight == other.weight and flow == other.flow and
               capacity == other.capacity and id == other.id;
    }
};

class Graph {
private:
    vector<Edge> edges;
    vector<int> potential;
    int start;
    int finish;
    int n;
    const int inf = (1 << 30);
    const Edge bad_edge = {-1, -1, -1, -1, -1, 0};
    const vector<Edge> bad_edge_vector = {bad_edge};

    vector<Edge> recover_path(vector<int>& d, vector<Edge>& parent) {
        potential = d;
        vector<Edge> answer;
        int v = finish;
        while (v != -1) {
            answer.push_back(parent[v]);
            v = parent[v].from;
        }
        if (answer.size() != 1) {
            answer.pop_back();
        }
        reverse(answer.begin(), answer.end());
        return answer;
    }

    vector<Edge> ford_bellman_path(bool decomposition) {

        vector<int> d(n, inf);
        vector<Edge> parent(n, bad_edge);
        d[start] = 0;

        for (size_t phase = 1; phase < n; ++phase) {
            for (Edge edge : edges) {
                if (edge.flow < edge.capacity or decomposition) {
                    if (!decomposition or (decomposition and edge.flow > 0)) {
                        if (d[edge.to] > d[edge.from] + edge.weight) {
                            parent[edge.to] = edge;
                            d[edge.to] = d[edge.from] + edge.weight;
                        }
                    }
                }
            }
        }

        return recover_path(d, parent);
    }

    vector<Edge> dijkstra_path(bool decomposition) {
        vector<int> d(n, inf);
        vector<Edge> parent(n, bad_edge);
        set<pair<int, int> > q;
        d[0] = 0;
        q.insert({d[0], 0});
        vector<vector<Edge> > gr(n);
        for (Edge edge : edges) {
            if (decomposition) {
                if (edge.flow > 0) {
                    gr[edge.from].push_back(edge);
                }
            } else {
                if (edge.flow < edge.capacity) {
                    gr[edge.from].push_back(edge);
                }
            }
        }

        while (!q.empty()) {
            int v = q.begin()->second;
            q.erase(q.begin());
            for (auto Edge : gr[v]) {
                int weight = Edge.weight + potential[Edge.to] - potential[Edge.from];
                if (d[Edge.from] + weight < d[Edge.to]) {
                    q.erase(make_pair(d[Edge.to], Edge.to));
                    d[Edge.to] = d[Edge.from] + weight;
                    parent[Edge.to] = Edge;
                    q.insert(make_pair(d[Edge.to], Edge.to));
                }
            }
        }

        return recover_path(d, parent);
    }

public:
    Graph(int _start, int _finish, int _n) : start(_start), finish(_finish), n(_n) {
        potential.resize(n);
    }

    void addEdge(int v, int w, int weight, int cap) {
        Edge new_edge = {v, w, weight, 0, cap, edges.size()};
        Edge reverse_edge = {w, v, -weight, 0, 0, edges.size() + 1};
        edges.push_back(new_edge);
        edges.push_back(reverse_edge);
    }

    int flow(int flow_size) {
        int answer = 0;
        auto Path = ford_bellman_path(false);
        while (Path != bad_edge_vector and answer < flow_size) {
            int diff = inf;
            for (auto edge : Path) {
                diff = min(diff, edge.capacity - edge.flow);
            }
            answer += diff;
            for (auto edge : Path) {
                edges[edge.id].flow += diff;
                edges[edge.id ^ 1].flow -= diff;
            }
            Path = dijkstra_path(false);
        }
        int res = 0;
        for (auto edge : edges) {
            if (edge.flow > 0) {
                res += edge.weight * edge.flow;
            }
        }
        if (res < flow_size) {
            res = -flow_size;
        }
        return res;
    }

    vector<vector<Edge> > decompose() {
        vector<vector<Edge> > answer;
        auto Path = ford_bellman_path(true);
        while (Path != bad_edge_vector) {
            int diff = inf;
            for (auto edge : Path) {
                diff = min(diff, edge.flow);
            }
            for (auto edge : Path) {
                edges[edge.id].flow -= diff;
                edges[edge.id ^ 1].flow += diff;
            }
            answer.push_back(Path);
            Path = dijkstra_path(true);
        }
        return answer;
    }

};

int main() {
    freopen("brides.in", "r", stdin);
    freopen("brides.out", "w", stdout);
    int n, m, k;
    cin >> n >> m >> k;
    Graph G(0, n - 1, n);
    for (int i = 0; i < m; ++i) {
        int a, b, t;
        cin >> a >> b >> t;
        --a;
        --b;
        G.addEdge(a, b, t, 1);
        G.addEdge(b, a, t, 1);
    }
    double f = G.flow(k);
    f /= k;
    auto vec = G.decompose();
    cout << fixed << setprecision(20);

    if (f != -1 and vec.size() == k) {
        cout << f << endl;
        for (auto path : vec) {
            cout << path.size() << " ";
            for (auto edge : path) {
                cout << edge.id / 4 + 1 << " ";
            }
            cout << endl;
        }
    } else {
        cout << -1;
    }
}