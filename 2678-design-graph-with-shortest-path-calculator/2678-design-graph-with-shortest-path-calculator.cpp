class Graph {
    private:
    std::unordered_map<int, std::vector<std::pair<int, int>>> adjacencyList;
public:
    Graph(int n, vector<vector<int>>& edges) {
         for (const auto& edge : edges) {
            int from = edge[0];
            int to = edge[1];
            int cost = edge[2];
            adjacencyList[from].push_back({to, cost});
        }
    }
    
    void addEdge(vector<int> edge) {
        int from = edge[0];
        int to = edge[1];
        int cost = edge[2];
        adjacencyList[from].push_back({to, cost});
    }
    
    int shortestPath(int node1, int node2) {
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;
        std::vector<int> distances(100, INT_MAX);
        pq.push({0, node1});
        distances[node1] = 0;

        while (!pq.empty()) {
            int currentNode = pq.top().second;
            int currDistance = pq.top().first;
            pq.pop();

            if (currentNode == node2) {
                return distances[node2];
            }

            if (currDistance > distances[currentNode]) {
                continue;
            }

            for (const auto& neighbor : adjacencyList[currentNode]) {
                int neighborNode = neighbor.first;
                int weight = neighbor.second;

                if (currDistance + weight < distances[neighborNode]) {
                    distances[neighborNode] = currDistance + weight;
                    pq.push({distances[neighborNode], neighborNode});
                }
            }    
        }   
        return -1;  
    }
};

/**
 * Your Graph object will be instantiated and called as such:
 * Graph* obj = new Graph(n, edges);
 * obj->addEdge(edge);
 * int param_2 = obj->shortestPath(node1,node2);
 */