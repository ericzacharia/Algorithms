from collections import defaultdict
from heapq import heappush, heappop
from time import perf_counter
from copy import deepcopy


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

    def ford_fulkerson(self, s, t): # Not Done  Implementing
        '''Edmonds-Karp Implementation
        
        If we found the path using BFS, then this is Edmonds Karp. Using DFS is standard FF.'''
        # Initialize residual graph as a copy of the orignial graph
        gr = deepcopy(self.graph)
        max_flow = 0  # initialize the flow
        parent = {}  # initialize hash table of parents from vertex s
        for vertex in self.graph:
            parent[vertex] = None

        while self.is_path_bfs(gr, s, t, parent):
            # find minimum residual capacity of the edges along the path from s to t found by BFS
            path_flow = float('inf')
            node = t
            while node != s:
                """Having trouble getting past this point. The following code would 
                pull the capacity from a matrix of capacities at (i,j) corresponding
                to the invcident vertices of the edge. I am using an adjacency list,
                so this needs to be changed accordingly - or the input needs to be modified."""
                path_flow = min(path_flow, gr[parent[node]][node])
                node = parent[node]
            max_flow += path_flow
        # update capacities in the residual graph and reverse edges along the path.
        v = t
        while v != s:
            u = parent[v]
            gr[u][v] -= path_flow
            gr[v][u] += path_flow
            v = parent[v]

        return max_flow

    def is_path_bfs(self, gr, s, t, parent):
        color = {}
        print(gr)
        for vertex in gr:
            color[vertex] = 'white'
            parent[vertex] = None
        color[s] = 'gray'

        queue = []  # FIFO
        queue.append(s)  # O(1)
        while queue:  # While queue is not empty
            u = queue.pop(0)  # O(1)
            for vertex, weight in gr[u]:  # O(E)
                if color[vertex] == 'white':
                    color[vertex] == 'gray'
                    parent[vertex] = u
                    queue.append(vertex)  # O(1)
            color[u] = 'black'
        for vertex in color:  # If every vertex is colored black, then a path exists
            if color[vertex] != 'black':
                return False
        return True

    def __repr__(self):
        return f'{dict(self.graph)}'


g = Graph()
g.add_edge('S', 'A', 16)
g.add_edge('S', 'C', 13)
g.add_edge('A', 'C', 10)
g.add_edge('C', 'A', 4)
g.add_edge('A', 'B', 12)
g.add_edge('B', 'C', 9)
g.add_edge('C', 'D', 14)
g.add_edge('D', 'T', 4)
g.add_edge('D', 'B', 7)
g.add_edge('B', 'T', 20)


print(f"{g.ford_fulkerson('S', 'T')} (computed using Ford-Fulkerson)")
