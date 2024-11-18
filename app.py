from flask import Flask, render_template, jsonify, request
import xml.etree.ElementTree as ET
from algorithms import bfs

app = Flask(__name__)




def process_osm_from_xml(file_path):
    """
    Process OSM XML to extract nodes and edges with relevant attributes.
    """
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Dictionaries to hold nodes and edges
    nodes = {}
    edges = []

    # Parse nodes
    for node in root.findall("node"):
        osmid = node.get("id")
        x = float(node.get("lon"))
        y = float(node.get("lat"))

        # Add node to dictionary with default parent as None
        nodes[osmid] = {
            "osmid": osmid,
            "x": x,
            "y": y,
            "parent": None
        }

    # Parse ways (edges)
    for way in root.findall("way"):
        refs = [nd.get("ref") for nd in way.findall("nd")]
        # Create edges from sequential pairs of refs
        for i in range(len(refs) - 1):
            edges.append({
                "u": refs[i],
                "v": refs[i + 1],
                "parent": None  # Placeholder for parent
            })

    return nodes, edges

@app.route("/")
def home():
    file_path = "dallasmap.osm"
    nodes, edges = process_osm_from_xml(file_path)
    return render_template("map.html", nodes=list(nodes.values()), edges=edges)


@app.route("/get-data")
def get_data():
    # Process the OSM file
    file_path = "dallasmap.osm"
    nodes, edges = process_osm_from_xml(file_path)
    return jsonify({"nodes": list(nodes.values()), "edges": edges})

@app.route("/run-algorithm", methods=["POST"])
def run_algorithm():
    data = request.json
    start_node = data["start_node"]
    target_node = data["target_node"]
    file_path = "dallasmap.osm"
    nodes, edges = process_osm_from_xml(file_path)
    found = bfs(start_node, target_node, list(nodes.values()), edges)
    print(" path found ", found)
    return jsonify({"found": found})



if __name__ == "__main__":
    app.run(debug=True)
