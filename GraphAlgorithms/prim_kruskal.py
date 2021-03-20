from collections import defaultdict
from heapq import heappush, heappop, heapify
from time import perf_counter


class Graph:
    '''This class represents a weighted undirected graph using adjacency list representation'''

    def __init__(self, graph):
        self.graph = graph  # adjacency list of vertices in Graph

    def add_edge(self, u, v, w):
        '''Add vertex u to v's adjacency list and vice versa in the Graph'''
        self.graph[v] = {u: w}
        self.graph[u] = {v: w}

    def mst_prim(self):
        '''Inputs: Graph (self) as a hash table, root vertex (r) as any type

        Returns the spanning tree as a hash table of vertices and the cost of 
        the Minimum Cost Spanning Tree (MST).

        Prim’s algorithm is similar to Dijkstra’s algorithm in that they both
        use a priority queue to select the next vertex to add to the growing
        graph (or growing spanning tree in this case).

        Time Complexity: O((V+E)logV) -> O(ElogV) since we know the input graph 
        is connected, which means we almost always have more edges than vertices 
        (unless our graph is a tree, then |E| = |V| - 1)'''
        mst = defaultdict(list)
        visited = set()
        r = list(self.graph.keys())[0]  # pick an arbitrary root vertex
        edges = [(weight, r, v) for v, weight in self.graph[r].items()]
        heapify(edges)
        mst_cost = 0
        while edges:
            weight, u, v = heappop(edges)
            if v not in visited:
                # mark u and v vertices incident to edge as visited
                visited.add(u)
                visited.add(v)
                mst[u].append(v)
                mst_cost += weight
                for v_next, weight in self.graph[v].items():
                    if v_next not in visited:
                        heappush(edges, (weight, v, v_next))

        return f'The cost of the MST is {mst_cost}, and the edges included in the spanning tree are {dict(mst)}'

    def mst_kruskal(self):  # Not Done implementing
        mst = defaultdict(list)
        edges = [(edge[2], edge[0], edge[1]) for edge in self.graph]
        heapify(edges)
        mst_cost = 0
        color = {}
        parent = {}
        discovery_time = {}
        for vertex in self.graph:  # Initialize arrays O(V)
            color[vertex] = 'white'
            parent[vertex] = None
        time = 0

        def is_cycle(u, v):  # Not Done implementing
            nonlocal discovery_time, parent
            # return discovery_time[v] < discovery_time[u] and parent[u] == v
            return False

        while edges:
            weight, u, v = heappop(edges)
            if not is_cycle(u, v):
                time += 1
                discovery_time[u] = time
                parent[v] = u
                color[u] = 'gray'
                mst[u].append(v)
                mst_cost += weight
        return f'The cost of the MST is {mst_cost}, and the edges included in the spanning tree are {dict(mst)}'

    def __repr__(self):
        return f'{dict(self.graph)}'


graph = {
    'A': {'B': 28, 'F': 10},
    'B': {'A': 28, 'C': 16, 'G': 14},
    'C': {'B': 16, 'D': 12},
    'D': {'E': 22, 'C': 12, 'G': 18},
    'E': {'D': 22, 'G': 24, 'F': 25},
    'F': {'E': 25, 'A': 10},
    'G': {'B': 14, 'D': 18, 'E': 24}
}

graph_kruskal = []
for u in graph:
    for v, w in graph[u].items():
        graph_kruskal.append((u, v, w))

g = Graph(graph)
gk = Graph(graph_kruskal)
print(f"{g.mst_prim()} (computed using Prim)")
print(f"{gk.mst_kruskal()} (Not done implementing Kruskal)")

#  --- Run each algorithm multiple times and time it ---
iterations = 500000
p_start = perf_counter()
for i in range(iterations):
    g.mst_prim()
p_stop = perf_counter()
k_start = perf_counter()
for i in range(iterations):
    gk.mst_kruskal()
k_stop = perf_counter()
#  --- End ---

print(f'{iterations} paths computed in {round((p_stop - p_start), 1)} seconds \
with Prim and {round((k_stop - k_start), 1)} seconds with Kruskal.')


# graph = {
#     'S': {'B': 2, 'C': 3, 'D': 3},
#     'B': {'S': 2, 'C': 4, 'E': 3},
#     'C': {'S': 3, 'B': 4, 'D': 5, 'E': 1, 'F': 6},
#     'D': {'S': 3, 'C': 5, 'F': 7},
#     'E': {'B': 3, 'C': 1, 'F': 8},
#     'F': {'C': 6, 'D': 7, 'E': 8, 'G': 9},
#     'G': {'F': 9}
# }
