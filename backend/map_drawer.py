import os
import json
import webbrowser
import numpy as np
import osmnx as ox

class MapDrawer:
    def __init__(self, place_name):
        self.graph = ox.graph_from_place(place_name, network_type="drive")

        self.nodes, self.edges = ox.graph_to_gdfs(self.graph)

        self.base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.frontend_dir = os.path.join(self.base_dir, "frontend")
        self.config_file = os.path.join(self.frontend_dir, "map_config.json")

        self.websocket_server = None

        if not os.path.exists(self.frontend_dir):
            raise RuntimeError(f"Frontend directory not found: {self.frontend_dir}")

        self.browser_opened = False
        self.server_url = "http://localhost:8000"

        edge_x = []
        edge_y = []
        for _, row in self.edges.iterrows():
            x, y = row["geometry"].xy
            edge_x.extend(x)
            edge_y.extend(y)
            edge_x.append(None)  # Separators
            edge_y.append(None)

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

        with open(self.config_file, 'w') as f:
            json.dump(map_config, f, indent=2)

        self.latest_coordinates = {
            'rear_lon': self.nodes['x'].mean(),
            'rear_lat': self.nodes['y'].mean(),
            'front_lon': self.nodes['x'].mean(),
            'front_lat': self.nodes['y'].mean(),
            'wheel_lon': self.nodes['x'].mean(),
            'wheel_lat': self.nodes['y'].mean()
        }

    def set_websocket_server(self, websocket_server):
        self.websocket_server = websocket_server

    def plot_bicycle(self, lat, lon, theta, alpha):
        lb, lw = 0.00015, 0.00008
        rear_lon, rear_lat = lon, lat

        front_lon = rear_lon + lb * np.cos(theta)
        front_lat = rear_lat + lb * np.sin(theta)

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

        if self.websocket_server:
            self.websocket_server.update_coordinates(self.latest_coordinates)
        else:
            print("Warning: WebSocket server not set, can't send coordinates")

    def show_map(self):
        if not self.browser_opened:
            webbrowser.open(self.server_url)
            self.browser_opened = True
            print(f"Browser opened to {self.server_url}")
