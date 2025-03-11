import osmnx as ox
import plotly.graph_objects as go
import numpy as np
import os
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import webbrowser
import json


class MapDrawer:
    def __init__(self, place_name):
        # Download street network
        self.graph = ox.graph_from_place(place_name, network_type="drive")

        # Get nodes and edges
        self.nodes, self.edges = ox.graph_to_gdfs(self.graph)

        # Create map
        self.fig = go.Figure()

        # Road coordinates
        edge_x = []
        edge_y = []

        for _, row in self.edges.iterrows():
            x, y = row["geometry"].xy
            edge_x.extend(x)
            edge_y.extend(y)
            edge_x.append(None)  # Separators
            edge_y.append(None)

        # Add roads to map
        self.fig.add_trace(go.Scattermapbox(
            lon=edge_x,
            lat=edge_y,
            mode='lines',
            line=dict(width=2, color='gray'),
            hoverinfo='none',
            name='Streets'
        ))

        # Add bicycle frame (initially empty)
        self.fig.add_trace(go.Scattermapbox(
            lon=[],
            lat=[],
            mode='lines+markers',
            marker=dict(size=10, color='blue'),
            line=dict(width=4, color='blue'),
            name='Bicycle Frame'
        ))

        # Add wheel (initially empty)
        self.fig.add_trace(go.Scattermapbox(
            lon=[],
            lat=[],
            mode='lines+markers',
            marker=dict(size=8, color='red'),
            line=dict(width=3, color='red'),
            name='Front Wheel'
        ))

        # Center map
        self.fig.update_layout(
            mapbox=dict(
                style="open-street-map",
                center=dict(
                    lon=self.nodes['x'].mean(),
                    lat=self.nodes['y'].mean()
                ),
                zoom=13
            ),
            margin={"r": 0, "t": 0, "l": 0, "b": 0},
            showlegend=True,
            height=800,
            width=1000,
            uirevision="constant"
        )

        # Server setup
        self.html_dir = os.path.dirname(os.path.abspath(__file__))  # Use script directory
        self.html_file = os.path.join(self.html_dir, "vehicle_map.html")
        self.json_file = os.path.join(self.html_dir, "coordinates.json")

        self.port = 8000
        self.server_thread = None
        self.server = None
        self.browser_opened = False

        # Create initial empty coordinates file
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

    def start_server(self):
        # Change to the script directory
        os.chdir(self.html_dir)

        # Create HTTP server with no-cache handler
        class NoCacheHTTPHandler(SimpleHTTPRequestHandler):
            def end_headers(self):
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                SimpleHTTPRequestHandler.end_headers(self)

        self.server = HTTPServer(("", self.port), NoCacheHTTPHandler)

        # Run on separate thread
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        print(f"Server started at http://localhost:{self.port}")

    def open_browser(self):
        if not self.browser_opened:
            webbrowser.open(f"http://localhost:{self.port}/{os.path.basename(self.html_file)}")
            self.browser_opened = True

    def plot_bicycle(self, lat, lon, theta, alpha):
        lb, lw = 0.0015, 0.0008  # axis lengths
        rear_lon, rear_lat = lon, lat  # rear axis start
        # rear axis end
        front_lon = rear_lon + lb * np.cos(theta)
        front_lat = rear_lat + lb * np.sin(theta)
        # front axis end
        wheel_lon = front_lon + lw * np.cos(alpha)
        wheel_lat = front_lat + lw * np.sin(alpha)

        # Latest coordinates
        self.latest_coordinates = {
            'rear_lon': rear_lon,
            'rear_lat': rear_lat,
            'front_lon': front_lon,
            'front_lat': front_lat,
            'wheel_lon': wheel_lon,
            'wheel_lat': wheel_lat
        }

        # Update coordinates.json file with latest position
        with open(self.json_file, 'w') as f:
            json.dump(self.latest_coordinates, f)
            f.flush()  # Ensure data is written immediately
            os.fsync(f.fileno())  # Force OS to write data to disk

        # Start server if needed
        if self.server_thread is None:
            self.start_server()
            self.open_browser()

    def show_map(self):
        # Generate initial HTML with Plotly map
        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Vehicle Tracking Map</title>
            <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
            <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate" />
            <meta http-equiv="Pragma" content="no-cache" />
            <meta http-equiv="Expires" content="0" />
        </head>
        <body>
            <div id="mapPlot"></div>
            <div id="status">Waiting for updates...</div>
            <script>
            var mapData = {self.fig.to_json()};
            Plotly.newPlot('mapPlot', mapData.data, mapData.layout);

            // Function to update coordinates
            function updateCoordinates() {{
                // Add random query parameter to prevent caching
                fetch('coordinates.json?_=' + new Date().getTime())
                    .then(response => response.json())
                    .then(coords => {{
                        // Update bicycle frame
                        Plotly.restyle('mapPlot', {{
                            'lon': [[coords.rear_lon, coords.front_lon]],
                            'lat': [[coords.rear_lat, coords.front_lat]]
                        }}, [1]);

                        // Update wheel
                        Plotly.restyle('mapPlot', {{
                            'lon': [[coords.front_lon, coords.wheel_lon]],
                            'lat': [[coords.front_lat, coords.wheel_lat]]
                        }}, [2]);

                        // Update status
                        document.getElementById('status').innerText = 
                            'Last update: ' + new Date().toLocaleTimeString() + 
                            ' - Position: ' + coords.rear_lat.toFixed(6) + ', ' + coords.rear_lon.toFixed(6);
                    }})
                    .catch(error => {{
                        console.error('Error fetching coordinates:', error);
                    }});
            }}

            // Update every 1 second
            setInterval(updateCoordinates, 1000);
            </script>
        </body>
        </html>
        """

        # Write to file
        with open(self.html_file, 'w') as f:
            f.write(html_content)

        # Start the server
        self.start_server()

        # Open in browser
        self.open_browser()

    def cleanup(self):
        if self.server:
            self.server.shutdown()
            if self.server_thread:
                self.server_thread.join()