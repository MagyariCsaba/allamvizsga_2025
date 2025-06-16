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

        self.message_counter = 0
        self.update_frequency = 1
        self.running = True


    def on_message(self, client_, userdata, message):
        try:
            message_content = message.payload.decode('utf-8')
            data = json.loads(message_content)

            self.message_counter += 1

            if self.message_counter % self.update_frequency == 0:
                ax = data['imuAccel'][0]
                ay = data['imuAccel'][1]
                theta = math.atan2(ay, ax)

                ax2 = data['imu2Accel'][0]
                ay2 = data['imu2Accel'][1]
                alpha = math.atan2(ay2, ax2)

                lat, lon = data['gpsPos'][0], data['gpsPos'][1]

                self.db_handler.save_message(data)

                self.map_drawer.plot_bicycle(lat, lon, theta, alpha)
                print(f"Map updated with: lat={lat}, lon={lon}, theta={theta}, alpha={alpha}")

        except (KeyError, ValueError) as e:
            print("Error processing message:", e)
        except Exception as e:
            print(f"Unexpected error: {e}")

    def start(self):
        try:
            self.client.loop_start()
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
