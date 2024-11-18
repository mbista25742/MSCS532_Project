def bfs(start_node, target_node, nodes, edges):
    # Create the graph as an adjacency list with string keys
    graph = {str(node['osmid']): [] for node in nodes}

    # Add edges to the graph using string keys
    for edge in edges:
        # Skip edges if 'u' or 'v' is missing or if either node is not in the nodes list
        if 'u' not in edge or 'v' not in edge:
            continue
        if str(edge['u']) not in graph:
            graph[str(edge['u'])] = []  # Add missing node
        if str(edge['v']) not in graph:
            graph[str(edge['v'])] = []  # Add missing node

        # Append the connections
        graph[str(edge['u'])].append(str(edge['v']))
        graph[str(edge['v'])].append(str(edge['u']))

    # Skip BFS if start or target nodes are missing
    if str(start_node) not in graph:
        print(f"Warning: Start node '{start_node}' is not in the graph. Skipping BFS.")
        return None
    if str(target_node) not in graph:
        print(f"Warning: Target node '{target_node}' is not in the graph. Skipping BFS.")
        return None

    # Initialize BFS
    queue = [str(start_node)]
    parent = {str(start_node): None}  # Keep track of parents to reconstruct the path

    while queue:
        current = queue.pop(0)

        # If we find the target node, stop and reconstruct the path
        if current == str(target_node):
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            path.reverse()
            print("path is ", path)
            return path  # Return the path from start to target

        # Process neighbors
        for neighbor in graph[current]:
            if neighbor not in parent:  # Check if the node has not been visited
                parent[neighbor] = current
                queue.append(neighbor)

    return None  # No path found
