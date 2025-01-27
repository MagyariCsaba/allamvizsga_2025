from database_handler import DatabaseHandler
from mqtt_client import MQTTClient
from map_drawer import MapDrawer
import numpy as np

if __name__ == "__main__":
    db_handler = DatabaseHandler()
    mqtt_client = MQTTClient(db_handler)
    map_drawer = MapDrawer("Marosvásárhely, Romania")
    map_drawer.draw_street_names()
    map_drawer.plot_bicycle(46.547, 24.551, np.pi / 2, np.pi / 3)
    map_drawer.show_map()
    mqtt_client.start()
