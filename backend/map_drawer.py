import os
import json
import threading
import webbrowser
import numpy as np
import osmnx as ox
import plotly.graph_objects as go
from http.server import HTTPServer, SimpleHTTPRequestHandler

class MapDrawer:
    def __init__(self, place_name):
        # Download street network
        self.graph = ox.graph_from_place(place_name, network_type="drive")

        # Get nodes and edges
        self.nodes, self.edges = ox.graph_to_gdfs(self.graph)

        # Setup paths
        self.base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.frontend_dir = os.path.join(self.base_dir, "frontend")
        self.json_file = os.path.join(self.frontend_dir, "coordinates.json")
        self.config_file = os.path.join(self.frontend_dir, "map_config.json")

        # Get WebSocket server reference (will be set later)
        self.websocket_server = None

        # Ensure frontend directory exists
        if not os.path.exists(self.frontend_dir):
            raise RuntimeError(f"Frontend directory not found: {self.frontend_dir}")

        # Server configuration
        self.port = 8000
        self.server_thread = None
        self.server = None
        self.browser_opened = False

        # Extract street coordinates for frontend
        edge_x = []
        edge_y = []
        for _, row in self.edges.iterrows():
            x, y = row["geometry"].xy
            edge_x.extend(x)
            edge_y.extend(y)
            edge_x.append(None)  # Separators
            edge_y.append(None)

        # Create map configuration for frontend
        map_config = {
            "streets": {
                "lon": edge_x,
                "lat": edge_y
            },
            "layout": {
                "mapbox": {
                    "style": "open-street-map",
                    "center": {
                        "lon": self.nodes['x'].mean(),
                        "lat": self.nodes['y'].mean()
                    },
                    "zoom": 13
                },
                "margin": {"r": 0, "t": 0, "l": 0, "b": 0},
                "showlegend": True,
                "height": 800,
                "width": 1000,
                "uirevision": "constant"
            }
        }

        # Save map configuration to file (only once at initialization)
        with open(self.config_file, 'w') as f:
            json.dump(map_config, f, indent=2)

        # Create initial coordinates file
        self.latest_coordinates = {
            'rear_lon': self.nodes['x'].mean(),
            'rear_lat': self.nodes['y'].mean(),
            'front_lon': self.nodes['x'].mean(),
            'front_lat': self.nodes['y'].mean(),
            'wheel_lon': self.nodes['x'].mean(),
            'wheel_lat': self.nodes['y'].mean()
        }

        # Save initial coordinates to file
        with open(self.json_file, 'w') as f:
            json.dump(self.latest_coordinates, f)

    def set_websocket_server(self, websocket_server):
        """Set the WebSocket server reference"""
        self.websocket_server = websocket_server

    def start_server(self):
        # Change to the frontend directory
        os.chdir(self.frontend_dir)

        # Create HTTP server with no-cache handler
        class NoCacheHTTPHandler(SimpleHTTPRequestHandler):
            def end_headers(self):
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                SimpleHTTPRequestHandler.end_headers(self)

        self.server = HTTPServer(("", self.port), NoCacheHTTPHandler)

        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        print(f"Server started at http://localhost:{self.port}")

    def open_browser(self):
        if not self.browser_opened:
            webbrowser.open(f"http://localhost:{self.port}/")
            self.browser_opened = True

    def plot_bicycle(self, lat, lon, theta, alpha):
        # axis lengths
        lb, lw = 0.00015, 0.00008
        # rear axis start
        rear_lon, rear_lat = lon, lat
        # rear axis end
        front_lon = rear_lon + lb * np.cos(theta)
        front_lat = rear_lat + lb * np.sin(theta)
        # front axis end
        wheel_lon = front_lon + lw * np.cos(alpha)
        wheel_lat = front_lat + lw * np.sin(alpha)

        self.latest_coordinates = {
            'rear_lon': rear_lon,
            'rear_lat': rear_lat,
            'front_lon': front_lon,
            'front_lat': front_lat,
            'wheel_lon': wheel_lon,
            'wheel_lat': wheel_lat
        }

        with open(self.json_file, 'w') as f:
            json.dump(self.latest_coordinates, f)
            f.flush()
            os.fsync(f.fileno())

        # Send via WebSocket if available
        if self.websocket_server:
            self.websocket_server.update_coordinates(self.latest_coordinates)

        # Start server if needed
        if self.server_thread is None:
            self.start_server()
            self.open_browser()




    def show_map(self):
        self.start_server()
        self.open_browser()

    def cleanup(self):
        if self.server:
            self.server.shutdown()
            if self.server_thread:
                self.server_thread.join()