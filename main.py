from backend.database_handler import DatabaseHandler
from backend.mqtt_client import MQTTClient
from backend.map_drawer import MapDrawer
from backend.websocket_server import WebSocketServer
import signal
import sys
import threading
import os
from http.server import HTTPServer, SimpleHTTPRequestHandler


def signal_handler(sig, frame):
    print("\nShutting down gracefully...")
    if mqtt_client:
        mqtt_client.stop()
    if websocket_server:
        websocket_server.stop()
    if http_server:
        http_server.shutdown()
    sys.exit(0)


def start_http_server(port, directory):
    os.chdir(directory)

    class NoCacheHTTPHandler(SimpleHTTPRequestHandler):
        def end_headers(self):
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            SimpleHTTPRequestHandler.end_headers(self)

    server = HTTPServer(("", port), NoCacheHTTPHandler)
    print(f"HTTP server started at http://localhost:{port}")

    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()

    return server


if __name__ == "__main__":
    HTTP_PORT = 8000
    WS_PORT = 8765

    db_handler = DatabaseHandler()
    websocket_server = WebSocketServer(port=WS_PORT)
    map_drawer = MapDrawer("Marosvásárhely, Romania")
    mqtt_client = None
    http_server = None

    base_dir = os.path.dirname(os.path.abspath(__file__))
    frontend_dir = os.path.join(base_dir, "frontend")

    http_server = start_http_server(HTTP_PORT, frontend_dir)

    map_drawer.server_url = f"http://localhost:{HTTP_PORT}"

    map_drawer.set_websocket_server(websocket_server)

    websocket_server.start()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        map_drawer.show_map()

        mqtt_client = MQTTClient(db_handler, map_drawer)

        print("Starting MQTT client. Press Ctrl+C to stop.")
        mqtt_client.start()

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if mqtt_client:
            mqtt_client.stop()
        if websocket_server:
            websocket_server.stop()
        if http_server:
            http_server.shutdown()