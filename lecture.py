import shutil

def line_break():
    terminal_width = shutil.get_terminal_size().columns
    line = '=' * terminal_width
    print(line)


##########
# Graphs #
##########

# Used to represent relationships between pairs of objects that consist vertices (nodes) and set of edges that connect the vertices

# Types of Graphs
# Directed Graph - One Way Street

class DirectedGraph:
    def __init__(self):
        self.graph = {}

    def add_edge(self, u, v):
        if u not in self.graph:
            self.graph[u] = []
        self.graph[u].append(v)

    def print_graph(self):
        for vertex, edges in self.graph.items():
            print(f'Vertex {vertex} -> {edges}')


dg = DirectedGraph()
dg.add_edge(0, 1)
dg.add_edge(0, 2)
dg.add_edge(1, 2)
dg.add_edge(2, 3)
dg.add_edge(3, 4)

dg.print_graph()


line_break()
# Undirected Graph - Two Way Streets
class UndirectedGraph:
    def __init__(self):
        self.graph = {}

    def add_edge(self, u, v):
        if u not in self.graph:
            self.graph[u] = []
        if v not in self.graph:
            self.graph[v] = []
        self.graph[u].append(v)
        self.graph[v].append(u)

    def print_graph(self):
        for vertex, edges in self.graph.items():
            print(f"Vertex {vertex}: {edges}")


ug = UndirectedGraph()
ug.add_edge(0, 1)
ug.add_edge(0, 2)
ug.add_edge(1, 2)
ug.add_edge(2, 3)
ug.add_edge(3, 4)

ug.print_graph()


# Adjacency Matrices
# Imagine you have a map with all the locations (vertices) you want to visit on your road trip. An adjacency matrix 
# is like a table where each row and column represent a location, and the entries indicate whether there's a 
# direct road (edge) between those locations. If there's a road connecting two locations, the corresponding entry is 1; otherwise, it's 0.
# Example adjacency matrix for a graph with 3 vertices


adj_matrix = [
    [0, 1, 1],
    [1, 0, 0],
    [1, 0, 0]
]



# Adjacency Lists
# An adjacency list is a more compact representation of a graph. It lists each vertex and its neighboring
# vertices. This representation is useful for sparse graphs where there are fewer connections between vertices.
# Example adjacency list for a graph with 4 vertices


adj_list = {
    1: [2, 3],
    2: [1, 3, 4],
    3: [1, 3],
    4: [2]
}


# Terminology

# Degree of a Vertex
# Number of edges connected to that vertex

# Path
# Sequence of vertices where each adjacent pair of vertices is connected by an edge aka the route from one vertex to another

# Cycle
# A path that starts and ends at the same vertex, traversing through a sequence of distinct vertices and edges without repetition

line_break()


class Graph:
    def __init__(self):
        self.vertices = {}

    # Method to add vertices to the graph
    def add_vertex(self, vertex):
        # If the vertex is not in the dictionary of vertices
        if vertex not in self.vertices:
            # Add the vertex to the dictionary with an empty list as the value
            self.vertices[vertex] = [] # Will be a list of all neighboring vertices

    # Method to add an edge between two vertices
    def add_edge(self, vertex1, vertex2):
        # Make sure that both vertices are in the graph
        if vertex1 in self.vertices and vertex2 in self.vertices:
            # Add an edge going from vertex1 to vertex2
            self.vertices[vertex1].append(vertex2)
            # Same edge going from vertex2 to vertex1 (undirected graph)
            self.vertices[vertex2].append(vertex1)

    # Method to display the vertices and their neighbors
    def display(self):
        # Loop through the vertices and edges of the dictionary
        for vertex, neighbors in self.vertices.items():
            print(f'Vertex {vertex} -> {neighbors}')

    # Method that will return a boolean on whether there is a path between two vertices
    def has_path(self, start, end, visited=None):
        # Initialize the visited set if not provided
        if visited is None:
            visited = set()
        # Mark the current vertex as visited
        visited.add(start)
        # Best case: If start and end are the same, return True
        if start == end:
            return True
        # Traverse through the neighbors of the current vertex
        for neighbor in self.vertices[start]:
            # If the neighbor has not been visited, recursively check if there is a path from neighbor to end
            if neighbor not in visited:
                if self.has_path(neighbor, end, visited):
                    return True
        # If no path is found from any neighbor to the end vertex, return False
        return False


social_network = Graph()
social_network.add_vertex('Brian')
social_network.add_vertex('Winter')
social_network.add_vertex('Chris')
social_network.add_vertex('Andy')
social_network.add_vertex('Victor')
social_network.add_vertex('Kayla')
social_network.add_vertex('Pheona')

social_network.add_edge('Winter', 'Chris')
social_network.add_edge('Chris', 'Andy')
social_network.add_edge('Andy', 'Victor')
social_network.add_edge('Kayla', 'Pheona')
social_network.add_edge('Pheona', 'Brian')
social_network.add_edge('Kayla', 'Chris')

social_network.display()


print("Path between Brian and Winter exists:", social_network.has_path('Brian', 'Winter'))
print("Path between Victor and Winter exists:", social_network.has_path('Victor', 'Winter'))


line_break()
line_break()

########################
# Dijkstra's Algorithm #
########################

# Dijkstra's Algorithm is a fundamental graph algorithm used to find the shortest path between two nodes in a weighted graph. It works well for graphs with non-negative edge weights.

# Imagine you're navigating through a city using Google Maps, and you want to find the shortest route from your current location to a destination. Dijkstra's Algorithm is like having a smart GPS that calculates the most efficient path for you, considering the distances between intersections and the traffic conditions.

# Here's how Dijkstra's Algorithm works:

# 1. **Initialization**: Start by assigning a tentative distance value to every node. Set the initial node's distance to 0 and all other nodes' distances to infinity.
# 2. **Visit Neighbors**: Explore all the neighboring nodes of the current node and update their tentative distances if a shorter path is found.
# 3. **Select Next Node**: Choose the node with the smallest tentative distance as the next current node.
# 4. **Repeat**: Repeat steps 2 and 3 until all nodes have been visited.

# The result is a shortest path tree from the source node to all other nodes in the graph.

import heapq

class Graph:
    def __init__(self):
        self.vertices = {}

    # Method to add vertices to the graph
    def add_vertex(self, vertex):
        # If the vertex is not in the dictionary of vertices
        if vertex not in self.vertices:
            # Add the vertex to the dictionary with an empty dictionary as the value
            self.vertices[vertex] = {} # Will be a dictionary of neighboring vertices as keys and weight as value

    # Method to add an edge between two vertices with weight
    def add_edge(self, vertex1, vertex2, weight):
        # Make sure that both vertices are in the graph
        if vertex1 in self.vertices and vertex2 in self.vertices:
            # Add an edge going from vertex1 to vertex2
            self.vertices[vertex1][vertex2] = weight
            # Same edge going from vertex2 to vertex1 (undirected graph)
            self.vertices[vertex2][vertex1] = weight

    # Method to find shortest path using the dijkstra
    def shortest_paths(self, start):
        # Initialize distances with infinity for all vertices except the start
        distances = {vertex: float('inf') for vertex in self.vertices}
        distances[start] = 0
        # Keep track of the paths
        paths = {vertex: [] for vertex in self.vertices}

        # Priority queue of (distance, vertex) pairs
        pq = [(0, start)]

        # While the priority queue is not empty
        while pq:
            # Get the vertex with the smallest distance from the priority queue
            current_distance, current_vertex = heapq.heappop(pq)

            # If the current distance is greater than the disatance already calculated for this vertex, skip this step
            if current_distance > distances[current_vertex]:
                continue

            # Explore the neighbors of the current vertex
            for neighbor, weight in self.vertices[current_vertex].items():
                # Calculate the distance to the neighbor through the current vertex
                distance = current_distance + weight

                # If this path to the neighbor is shorter than any previously found path, update the distance
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    # Update the path
                    paths[neighbor] = paths[current_vertex] + [current_vertex]
                    # Add the neighbor to the priority queue with its updated distance
                    heapq.heappush(pq, (distance, neighbor))
        # After the while loop has finished, return the distances dictionary
        return distances, paths


# Example GPS


gps = Graph()

gps.add_vertex('Home')
gps.add_vertex('Office')
gps.add_vertex('Gym')
gps.add_vertex('Store')
gps.add_vertex('Pub')

gps.add_edge('Home', 'Gym', 5)
gps.add_edge('Home', 'Office', 2)
gps.add_edge('Office', 'Gym', 4)
gps.add_edge('Store', 'Gym', 6)
gps.add_edge('Office', 'Store', 1)
gps.add_edge('Store', 'Pub', 3)

home_distances = gps.shortest_paths('Home')
print('Shortest distances from Home:', home_distances)

line_break()
store_distances = gps.shortest_paths('Store')
print('Shortest distances from Store:', store_distances)