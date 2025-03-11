import json
import paho.mqtt.client as mqtt
import math
import time


class MQTTClient:
    def __init__(self, db_handler, map_drawer):
        self.client = mqtt.Client("PythonSubscriber")
        self.client.on_message = self.on_message
        self.client.connect("localhost", port=1883)
        self.client.subscribe("eesTopic")
        self.db_handler = db_handler
        self.map_drawer = map_drawer

        # To track if we need to update the display
        self.counter = 0
        self.update_frequency = 300  # Update every 20 messages
        self.running = True

    def on_message(self, client_, userdata, message):
        try:
            # Decode message
            message_content = message.payload.decode('utf-8')
            data = json.loads(message_content)

            # Calculate orientation angle from first IMU
            ax = data['imuAccel'][0]
            ay = data['imuAccel'][1]
            theta = math.atan2(ay, ax)

            # Calculate second angle from second IMU
            ax2 = data['imu2Accel'][0]
            ay2 = data['imu2Accel'][1]
            alpha = math.atan2(ay2, ax2)

            # Get GPS coordinates
            lat, lon = data['gpsPos'][0], data['gpsPos'][1]

            # Save to database
            self.db_handler.save_message(data)

            # Update map with every message
            self.map_drawer.plot_bicycle(lat, lon, theta, alpha)
            print(f"Map updated with: lat={lat}, lon={lon}, theta={theta}, alpha={alpha}")

        except (KeyError, ValueError) as e:
            print("Error processing message:", e)
        except Exception as e:
            print(f"Unexpected error: {e}")

    def start(self):
        try:
            self.client.loop_start()
            # Keep the main thread running
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopping MQTT client...")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        self.client.loop_stop()
        self.client.disconnect()
        self.map_drawer.cleanup()  # Clean up the server