import json
import paho.mqtt.client as mqtt
from database_handler import DatabaseHandler


class MQTTClient:
    def __init__(self, db_handler):
        self.client = mqtt.Client("PythonSubscriber")
        self.client.on_message = self.on_message
        self.client.connect("localhost", port=1883)
        self.client.subscribe("eesTopic")
        self.db_handler = db_handler

    def on_message(self, client_, userdata, message):
        try:
            message_content = message.payload.decode('utf-8')
            data = json.loads(message_content)
            self.db_handler.save_message(data)
            print(f"Message received and saved: {message_content}")
        except (KeyError, ValueError) as e:
            print("Error processing message:", e)

    def start(self):
        self.client.loop_forever()