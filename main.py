from backend import websocket_server
from backend.database_handler import DatabaseHandler
from backend.mqtt_client import MQTTClient
from backend.map_drawer import MapDrawer
from backend.websocket_server import WebSocketServer
import signal
import sys


def signal_handler(sig, frame):
    print("\nShutting down gracefully...")
    if mqtt_client:
        mqtt_client.stop()
    if websocket_server:
        websocket_server.stop()
    sys.exit(0)


if __name__ == "__main__":

    # Initialize components
    db_handler = DatabaseHandler()
    websocket_server = WebSocketServer(port=8765)
    map_drawer = MapDrawer("Marosvásárhely, Romania")
    mqtt_client = None

    # Connect the WebSocket server to the map drawer
    map_drawer.set_websocket_server(websocket_server)

    # Start the WebSocket server
    websocket_server.start()

    # Register signal handler for clean shutdown
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
        # Clean up
        if mqtt_client:
            mqtt_client.stop()
        if websocket_server:
            websocket_server.stop()