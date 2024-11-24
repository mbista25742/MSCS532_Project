import networkx as nx
import heapq
from collections import deque


# A* Search Algorithm
def a_star(G, start, end):
    open_set = []
    heapq.heappush(open_set, (0, start))  # (f, node)
    came_from = {}
    g_score = {node: float('inf') for node in G.nodes}
    f_score = {node: float('inf') for node in G.nodes}
    g_score[start] = 0
    f_score[start] = heuristic(G, start, end)

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == end:
            return reconstruct_path(came_from, current)

        for neighbor in G.neighbors(current):
            tentative_g_score = g_score[current] + G[current][neighbor].get('weight', 1)
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(G, neighbor, end)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return []

# Heuristic function for A* (Euclidean distance)
def heuristic(G, node, goal):
    x1, y1 = G.nodes[node]['x'], G.nodes[node]['y']
    x2, y2 = G.nodes[goal]['x'], G.nodes[goal]['y']
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

# Reconstruct the path from the came_from dictionary
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# Dijkstra's Algorithm
def dijkstra(G, start, end):
    # Initialize the priority queue and dictionaries
    open_set = []
    heapq.heappush(open_set, (0, start))  # (distance, node)
    distance = {node: float('inf') for node in G.nodes}
    previous_node = {node: None for node in G.nodes}
    distance[start] = 0

    while open_set:
        current_distance, current_node = heapq.heappop(open_set)

        # If we reached the end node, reconstruct the path
        if current_node == end:
            return reconstruct_path_from_previous(previous_node, end)

        # Explore the neighbors
        for neighbor in G.neighbors(current_node):
            edge_weight = G[current_node][neighbor].get('weight', 1)
            new_distance = current_distance + edge_weight

            # If a shorter path to the neighbor is found, update
            if new_distance < distance[neighbor]:
                distance[neighbor] = new_distance
                previous_node[neighbor] = current_node
                heapq.heappush(open_set, (new_distance, neighbor))

    return []

# Reconstruct the path from the previous_node dictionary
def reconstruct_path_from_previous(previous_node, current):
    path = [current]
    while previous_node[current] is not None:
        current = previous_node[current]
        path.append(current)
    path.reverse()
    return path


# Breadth-First Search (BFS)
def bfs(G, start, end):
    # Queue to hold nodes to explore
    queue = deque([start])
    # Dictionary to track the node from which we reached the current node
    came_from = {start: None}
    
    while queue:
        current = queue.popleft()

        # If we reached the end node, reconstruct the path
        if current == end:
            return reconstruct_path_b(came_from, end)

        # Explore neighbors
        for neighbor in G.neighbors(current):
            if neighbor not in came_from:  # If neighbor is not visited
                came_from[neighbor] = current
                queue.append(neighbor)
    
    # If no path is found, return an empty list
    return []

# Reconstruct the path from the came_from dictionary
def reconstruct_path_b(came_from, current):
    path = [current]
    while current in came_from and came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# Depth-First Search (DFS) (New Code)
def dfs(G, start, end):
    stack = [start]
    came_from = {start: None}

    while stack:
        current = stack.pop()

        if current == end:
            return reconstruct_path_d(came_from, end)

        for neighbor in G.neighbors(current):
            if neighbor not in came_from:
                came_from[neighbor] = current
                stack.append(neighbor)

    return []

def reconstruct_path_d(came_from, current):
    path = [current]
    while current in came_from and came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
