import osmnx as ox
import random
from flask import Flask, render_template, jsonify
import geopandas as gpd
from shapely.geometry import Point
from algorithms import a_star, dijkstra, bfs, dfs
import networkx as nx

app = Flask(__name__)

# Function to load OSM data from file or from place name
def load_osm_data(osm_file=None, place_name=None):
    if osm_file:
        G = ox.graph_from_xml(osm_file)
    elif place_name:
        G = ox.graph_from_place(place_name, network_type='all')
    else:
        raise ValueError("Must provide either osm_file or place_name")

    # Convert the graph to undirected
    G = G.to_undirected()
    return G

# Load the graph (replace with your OSM file path or place name)
osm_file_path = './map_data/dallas.osm'  
G = load_osm_data(osm_file=osm_file_path) 

# Get the center of the map (the geographical center of the OSM data)
nodes_gdf = ox.graph_to_gdfs(G, nodes=True, edges=False)

# Reproject to a projected CRS (e.g., UTM) for accurate centroid calculation
nodes_gdf = nodes_gdf.to_crs(epsg=3395)  # EPSG:3395 is the World Mercator projection

# Now calculate the centroid in the projected CRS
center_x = nodes_gdf['geometry'].centroid.x.mean()
center_y = nodes_gdf['geometry'].centroid.y.mean()

# Convert the centroid coordinates back to a Shapely Point
centroid_point = Point(center_x, center_y)

# Create a GeoDataFrame from the Point geometry
centroid_gdf = gpd.GeoDataFrame(geometry=[centroid_point], crs="EPSG:3395")

# Reproject the centroid GeoDataFrame back to the geographic CRS (lat, lon)
centroid_gdf = centroid_gdf.to_crs(epsg=4326)  # EPSG:4326 is the geographic CRS

# Extract the latitude and longitude of the centroid
center_lat, center_lon = centroid_gdf.geometry.iloc[0].y, centroid_gdf.geometry.iloc[0].x

import osmnx as ox
from shapely.geometry import Point

# Function to get the nearest node in the graph from a coordinate
def get_nearest_node(G, lat, lon):
    return ox.distance.nearest_nodes(G, X=lon, Y=lat)

# Update to get random nodes to get the nearest connected node to the random start and end locations
def get_random_nodes():
    while True:
        # Select two random points (coordinates) within the map bounds
        lat_a = random.uniform(center_lat - 0.01, center_lat + 0.01)
        lon_a = random.uniform(center_lon - 0.01, center_lon + 0.01)
        lat_b = random.uniform(center_lat - 0.01, center_lat + 0.01)
        lon_b = random.uniform(center_lon - 0.01, center_lon + 0.01)

        # Get nearest nodes for each random point
        node_a = get_nearest_node(G, lat_a, lon_a)
        node_b = get_nearest_node(G, lat_b, lon_b)

        # Check if the nodes are connected
        if nx.has_path(G, node_a, node_b):
            return node_a, node_b, (lat_a, lon_a), (lat_b, lon_b)

@app.route('/')
def index():
    node_a, node_b, start_coords, end_coords = get_random_nodes()

    # Get the latitude and longitude of the nodes
    node_a_data = G.nodes[node_a]
    node_b_data = G.nodes[node_b]

    return render_template('index.html', node_a=node_a, node_b=node_b,
                           start_coords=start_coords, end_coords=end_coords,
                           center_lat=center_lat, center_lon=center_lon)


@app.route('/shortest_path/<start_node>/<end_node>/<algo>', methods=['GET'])
def get_shortest_path(start_node, end_node, algo):
    start_node = int(start_node)
    end_node = int(end_node)
    
    if algo == 'a_star':
        path = a_star(G, start_node, end_node)
    elif algo == 'dijkstra':
        path = dijkstra(G, start_node, end_node)
    elif algo == 'breadth_first':
        path = bfs(G, start_node, end_node)
    elif algo == 'depth_first':
        path = dfs(G, start_node, end_node)
    else:
        return jsonify({'error': 'Unknown algorithm'}), 400
    
    # Debugging: log the returned path
    if path is None:
        return jsonify({'error': 'No path found'}), 404
    
    # Convert the node IDs in the path to (lat, lon) coordinates
    path_coords = []
    for node in path:
        node_data = G.nodes[node]
        path_coords.append((node_data['y'], node_data['x']))  # (lat, lon)

    return jsonify(path_coords)


if __name__ == '__main__':
    app.run(debug=True)
