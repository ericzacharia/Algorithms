from collections import defaultdict
from heapq import heappush, heappop
from time import perf_counter


class Graph:
    '''This class represents a weighted directed graph using adjacency list representation'''

    def __init__(self):
        self.graph = defaultdict(list)  # adjacency list of vertices in Graph
        self.weights = {}

    def add_edge(self, u, v, w):
        '''Add (vertex, weight) tuple (v, w) to u's adjacency list in the Graph.
        Also initialize adjacency list for v if necessary.'''
        self.graph[u].append((v, w))

        if v not in self.graph:
            self.graph[v] = []

        self.weights[(u, v)] = w  # Add to hash table of edge weights

    def dijkstra(self, s):
        '''Inputs: Graph (self) as a hash table, source vertex (s) as any type

        Returns the hash table of vertices and the length of their shortest 
        paths from source vertex s.

        This algorithm cannot handle graphs with negative edges. Dijkstra is 
        cost efficient because all edges are non-negative.

        This Dijkstra algorithm implements Python's heapq module, which takes 
        advantage of min heap operations with time complexities comparable to
        the Fibonacci heap, which is one of the most efficient implementations
        of a min heap.

        Time complexity: O((V + E) * log(V))'''
        # Initialize-Single-Source ------------
        distance = {}  # initialize hash table of distances from vertex s
        parent = {}  # initialize hash table of parents from vertex s
        for vertex in self.graph:
            distance[vertex] = float('inf')
            parent[vertex] = None
        distance[s] = 0
        # -------------------------------------

        # Implement min-priority queue with a Fibonacci heap O(log(V))
        S = set()  # Set of vertices whose shortest paths from s are finalized
        # Initially all the vertices are placed in a priority queue
        Q = [(0, s)]  # Initialize priority queue of (distance, vertex) tuples

        while Q:  # While pq is not empty
            # O(logV) to extract min distance from min heap. Done V times -> O(VlogV)
            u = heappop(Q)[1]
            S = S.union(u)  # (CLRS) S seems unnecessary given Q
            for u, v in self.weights:
                # Relax ------------------------------------------
                if distance[v] > distance[u] + self.weights[u, v]:
                    distance[v] = distance[u] + self.weights[u, v]
                    parent[v] = u
                # ------------------------------------------------
                    # push (distance, verex) into min-priority queue and heapify
                    heappush(Q, (distance[v], parent))  # O(logV). Done E times -> O(ElogV)
        
        return f'The shortest distance from {s} to each vertex is {distance}'

    def bellman_ford(self, s):
        '''Inputs: Graph (self) as a hash table, source vertex (s) as any type

        Returns the hash table of vertices and the length of their shortest 
        paths from source vertex s.

        This differs from Dijkstra's algorithm in that it works on graphs with 
        negative edges, but with a different time complexity.

        This algorithm cannot handle graphs with negative cycles.

        Time complexity: O(V * E)'''
        V = len(self.graph.keys())  # number of vertices
        distance = {}  # initialize hash table of distances from vertex s
        for vertex in self.graph:
            distance[vertex] = float('inf')

        distance[s] = 0  # distance from vertex s to vertex s is 0

        for _ in range(V - 1):  # O(V*E)
            for u, v in self.weights:
                distance[v] = min(  # Relax
                    distance[v], distance[u] + self.weights[u, v])

        return f'The shortest distance from {s} to each vertex is {distance}'

    def __repr__(self):
        return f'{dict(self.graph)}'


g = Graph()
g.add_edge('S', 'B', 4)
g.add_edge('S', 'C', 2)
g.add_edge('C', 'B', 1)
g.add_edge('B', 'C', 3)
g.add_edge('B', 'E', 3)
g.add_edge('C', 'E', 5)
g.add_edge('C', 'D', 4)
g.add_edge('E', 'D', 1)
g.add_edge('B', 'D', 2)

print(f"{g.dijkstra('S')} (computed using Dijkstra)")
print(f"{g.bellman_ford('S')} (computed using Bellman-Ford)")

#  --- Run each algorithm multiple times and time it ---
iterations = 500000
d_start = perf_counter()
for i in range(iterations):
    g.dijkstra('S')
d_stop = perf_counter()
b_start = perf_counter()
for i in range(iterations):
    g.bellman_ford('S')
b_stop = perf_counter()
#  --- End ---

print(f'{iterations} paths computed in {round((d_stop - d_start), 1)} seconds \
with Dijkstra and {round((b_stop - b_start), 1)} seconds with Bellman-Ford.')

print('-------Below tests a graph with negative edge weights---------')

gb = Graph()
gb.add_edge('S', 'A', 10)
gb.add_edge('S', 'E', 8)
gb.add_edge('E', 'D', 1)
gb.add_edge('D', 'A', -4)
gb.add_edge('D', 'C', -1)
gb.add_edge('A', 'C', 2)
gb.add_edge('C', 'B', -2)
gb.add_edge('B', 'A', 1)
print(f"Graph to be used with Bellman-Ford's Algorithm: {gb}")
print(f"{gb.bellman_ford('S')}")
