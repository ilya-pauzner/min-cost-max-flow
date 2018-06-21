#include <algorithm>
#include <iomanip>
#include <iostream>
#include <set>


using std::cin;
using std::cout;
using std::fixed;
using std::make_pair;
using std::min;
using std::pair;
using std::reverse;
using std::set;
using std::setprecision;
using std::vector;

class Graph;

struct Edge {
    size_t from;
    size_t to;
    int weight;
    int flow;
    int capacity;
    size_t id;

    bool operator==(const Edge& other) const {
        return from == other.from && to == other.to && weight == other.weight && flow == other.flow &&
               capacity == other.capacity && id == other.id;
    }
};

class Graph {
private:
    vector<Edge> edges;
    vector<int> potential;
    size_t start;
    size_t finish;
    size_t n;
    const unsigned int inf = (1 << 30);
    const Edge bad_edge = {inf, inf, 0, 0, 0, inf};
    const vector<Edge> bad_edge_vector = {bad_edge};

    vector<Edge> recover_path(vector<int>& distances, vector<Edge>& parent) {
        potential = distances;
        vector<Edge> answer;
        size_t v = finish;
        while (v != inf) {
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

        vector<int> distances(n, inf);
        vector<Edge> parent(n, bad_edge);
        distances[start] = 0;

        for (size_t phase = 1; phase < n; ++phase) {
            for (Edge edge : edges) {
                if (edge.flow < edge.capacity || decomposition) {
                    if (!decomposition || edge.flow > 0) {
                        if (distances[edge.to] > distances[edge.from] + edge.weight) {
                            parent[edge.to] = edge;
                            distances[edge.to] = distances[edge.from] + edge.weight;
                        }
                    }
                }
            }
        }

        return recover_path(distances, parent);
    }

    vector<Edge> dijkstra_path(bool decomposition) {
        vector<int> distances(n, inf);
        vector<Edge> parent(n, bad_edge);
        set<pair<int, int> > q;
        distances[0] = 0;
        q.insert({distances[0], 0});
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
                if (distances[Edge.from] + weight < distances[Edge.to]) {
                    q.erase(make_pair(distances[Edge.to], Edge.to));
                    distances[Edge.to] = distances[Edge.from] + weight;
                    parent[Edge.to] = Edge;
                    q.insert(make_pair(distances[Edge.to], Edge.to));
                }
            }
        }

        return recover_path(distances, parent);
    }

public:
    Graph(size_t _start = 0, size_t _finish = 0, size_t _n = 0) : start(_start), finish(_finish), n(_n) {
        potential.resize(n);
    }

    void addEdge(size_t v, size_t w, int weight, int cap) {
        Edge new_edge = {v, w, weight, 0, cap, edges.size()};
        Edge reverse_edge = {w, v, -weight, 0, 0, edges.size() + 1};
        edges.push_back(new_edge);
        edges.push_back(reverse_edge);
    }

    int flow(int flow_size) {
        int answer = 0;
        auto Path = ford_bellman_path(false);
        while (Path != bad_edge_vector && answer < flow_size) {
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
        auto path = ford_bellman_path(true);
        while (path != bad_edge_vector) {
            int diff = inf;
            for (auto edge : path) {
                diff = min(diff, edge.flow);
            }
            for (auto edge : path) {
                edges[edge.id].flow -= diff;
                edges[edge.id ^ 1].flow += diff;
            }
            answer.push_back(path);
            path = dijkstra_path(true);
        }
        return answer;
    }

};

Graph read(Graph* G, int* k) {
    freopen("brides.in", "r", stdin);
    freopen("brides.out", "w", stdout);
    size_t n, m;
    *k = 0;
    cin >> n >> m >> *k;
    *G = Graph(0, n - 1, n);
    for (int i = 0; i < m; ++i) {
        size_t a, b;
        int t;
        cin >> a >> b >> t;
        --a;
        --b;
        G->addEdge(a, b, t, 1);
        G->addEdge(b, a, t, 1);
    }
}

vector<vector<Edge> > solve(vector<vector<Edge> >* answer, Graph* G, int k, double* flow) {
    *flow = G->flow(k);
    *flow /= k;
    *answer = G->decompose();
}

void write(const vector<vector<Edge> >& answer, int k, double flow) {
    cout << fixed << setprecision(20);
    size_t edge_redundancy = 4;
    if (flow != -1 && answer.size() == k) {
        cout << flow << "\n";
        for (auto path : answer) {
            cout << path.size() << " ";
            for (auto edge : path) {
                cout << edge.id / edge_redundancy + 1 << " ";
            }
            cout << "\n";
        }
    } else {
        cout << -1;
    }
}

int main() {
    int k = 0;
    Graph G;
    read(&G, &k);
    double flow = 0;
    vector<vector<Edge> > answer;
    solve(&answer, &G, k, &flow);
    write(answer, k, flow);
}
