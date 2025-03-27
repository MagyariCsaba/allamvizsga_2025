from backend.database_handler import DatabaseHandler
from backend.mqtt_client import MQTTClient
from backend.map_drawer import MapDrawer
import signal
import sys


def signal_handler(sig, frame):
    print("\nShutting down gracefully...")
    if mqtt_client:
        mqtt_client.stop()
    sys.exit(0)


if __name__ == "__main__":
    # Initialize components
    db_handler = DatabaseHandler()
    map_drawer = MapDrawer("Marosvásárhely, Romania")
    mqtt_client = None

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