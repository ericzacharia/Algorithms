from collections import defaultdict
from heapq import heappush, heappop
from time import perf_counter


class Graph:
    '''This class represents a weighted directed graph using adjacency list representation'''

    def __init__(self):
        self.graph = defaultdict(list)  # adjacency list of vertices in Graph

    def add_edge(self, u, v):
        '''Add (vertex, weight) tuple (v, w) to u's adjacency list in the Graph.
        Also initialize adjacency list for v if necessary.'''
        self.graph[u].append(v)

        if v not in self.graph:
            self.graph[v] = []

    def bfs(self, s, t):
        '''Inputs: Graph (self) as a hash table, source vertex (s), and target 
        vertex (t).

        bfs prints out the path from s to t, which solves the single source 
        shortest path problem.

        Time complexity: O(V +  E)'''
        # Initialize arrays O(V)
        color = {}
        # distance = {}
        parent = {}
        for vertex in self.graph:
            color[vertex] = 'white'
            # distance[vertex] = float('inf')
            parent[vertex] = None
        # -------------------------
        # Set color and distance for source vertex O(1)
        color[s] = 'gray'
        # distance[s] = 0
        # -------------------------

        queue = []  # FIFO
        queue.append(s)  # O(1)
        while queue:  # While queue is not empty
            u = queue.pop(0)  # O(1)
            # The sum of the lengths of all adj lists = # of edges in G
            for vertex in self.graph[u]:  # O(E)
                if color[vertex] == 'white':
                    color[vertex] == 'gray'
                    # distance[vertex] += 1
                    parent[vertex] = u
                    queue.append(vertex)  # O(1)
            color[u] = 'black'

        shortest_path = []
        self.shortest_path_bfs(s, t, parent, shortest_path)

        return shortest_path

    def shortest_path_bfs(self, s, t, parent, shortest_path):
        if t == s:
            shortest_path.append(s)
        elif parent[t] is None:
            print(f"No path from {s} to {t} exists")
        else:
            self.shortest_path_bfs(s, parent[t], parent, shortest_path)
            shortest_path.append(t)

    def dfs(self):
        '''Inputs: Graph (self) as a hash table.

        Returns the topologically sorted order of the graph, vertex discovery 
        times, their adjacency list finish times, the number of connected
        components, a boolean value representing whether or not a cycle exists,
        and a list of shortest paths ??
        .

        Time complexity: Theta(V +  E)'''
        color = {}
        parent = {}
        discovery_time = {}
        finish_time = {}
        topological_sort = []
        connected_components = 0
        is_cyclic = False

        for vertex in self.graph:  # Initialize arrays O(V)
            color[vertex] = 'white'
            parent[vertex] = None
        time = 0

        def dfs_visit(u):
            nonlocal time, parent, discovery_time, finish_time, topological_sort, is_cyclic
            time += 1
            discovery_time[u] = time
            color[u] = 'gray'

            for v in self.graph[u]:  # O(E) (number of edges in u's adj list)
                if color[v] != 'white':  # If v has been discovered
                    # If vertex v was discovered before vertex u AND v is the parent of u,
                    if discovery_time[v] < discovery_time[u] and parent[u] == v:
                        is_cyclic = True  # Then a cycle exists
                else:  # v has not previously been discovered
                    parent[v] = u
                    dfs_visit(v)

            color[u] = 'black'
            time += 1
            finish_time[u] = time
            topological_sort.append(u)

        for vertex in self.graph:  # O(E) No more because of color array
            if color[vertex] == 'white':  # discovered is gray/black
                # Num of connected components = Num times dfs_visit is called from here
                dfs_visit(vertex)
                connected_components += 1

        topological_sort.reverse()  # O(V)
        # is_bipartite = self.is_bipartite_dfs()  # O(V + E)
        is_bipartite = True  # placeholder
        return topological_sort, discovery_time, finish_time, connected_components, is_cyclic, is_bipartite

        # def dfs_shortest_path(self, u, v):  # Not done implementing
        #     path_stack = []
        #     path_visited = {}
        #     paths = []
        #     for vertex in self.graph:  # O(V)
        #         path_visited[vertex] = False
        #     return self.dfs_shortest_path_recursive(u, v, path_stack, path_visited, paths)

        # # Not done implementing
        # def dfs_shortest_path_recursive(self, u, v, path_stack, path_visited, paths):

        #     path_visited[u] = True
        #     path_stack.append(u)
        #     if u == v:
        #         print(paths)
        #         paths.append(path_stack)
        #     else:
        #         for vertex in self.graph[u]:
        #             if not path_visited[vertex]:
        #                 self.dfs_shortest_path_recursive(
        #                     vertex, v, path_stack, path_visited, paths)
        #     path_stack.pop()
        #     path_visited[u] = False
        #     print(paths)
        #     return paths

    # def is_bipartite_dfs(self):          # Not done implementing

    #     color = {}
    #     visited = {}

    #     for vertex in self.graph:  # Initialize arrays O(V)
    #         color[vertex] = 'red'
    #         visited[vertex] = False
    #     print(list(visited.keys())[0])
    #     first_vertex = list(visited.keys())[0]
    #     visited[first_vertex] = True

    #     return self.is_bipartite_dfs_recursive(first_vertex, visited, color)

    # def is_bipartite_dfs_recursive(self, v, visited, color):         ## Not done implementing

    #     for u in self.graph[v]:
    #         if not visited[u]:
    #             print(color)
    #             if color[v] == 'red':
    #                 color[u] == 'blue'
    #             else:
    #                 color[u] == 'red'
    #             if not self.is_bipartite_dfs_recursive(u, visited, color):
    #                 return False
    #         elif color[u] == color[v]:
    #             return False
    #     return True

    def __repr__(self):
        return f'{dict(self.graph)}'


g = Graph()
g.add_edge('A', 'B')
g.add_edge('A', 'C')
g.add_edge('B', 'D')
g.add_edge('C', 'D')
g.add_edge('C', 'E')
g.add_edge('D', 'F')
g.add_edge('E', 'F')
g.add_edge('F', 'E')

source = 'A'
target = 'F'
shortest_path = g.bfs(source, target)
print(f"BFS: Shortest Path from {source} to {target}: {shortest_path}")
topological_sort, discovery_time, finish_times, connected_components, is_cyclic, is_bipartite = g.dfs()
shortest_path = 1
# shortest_path = g.dfs_shortest_path('A', 'F')
print(
    f"DFS: Topologically Sorted Order: {topological_sort}, Discovery Times: {discovery_time} Finish Times: {finish_times}, Number of Connected Components: {connected_components}, Cyclic? {is_cyclic}, Paths: {shortest_path}(unfinished), Bipartite? {is_bipartite}(unfinished)")
