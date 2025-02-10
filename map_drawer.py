import osmnx as ox
import matplotlib.pyplot as plt
import numpy as np


class MapDrawer:
    def __init__(self, place_name):
        self.graph = ox.graph_from_place(place_name, network_type="drive")
        self.fig, self.ax = ox.plot_graph(
            self.graph, node_size=0, bgcolor='white', edge_linewidth=2,
            figsize=(50, 50), edge_color='gray', show=False, close=False
        )

    def draw_street_names(self):
        displayed_names = set()
        for u, v, data in self.graph.edges(data=True):
            if 'name' in data:
                street_name = data['name']
                if isinstance(street_name, list):
                    street_name = ", ".join(street_name)
                if street_name not in displayed_names:
                    x = (self.graph.nodes[u]['x'] + self.graph.nodes[v]['x']) / 2
                    y = (self.graph.nodes[u]['y'] + self.graph.nodes[v]['y']) / 2
                    x_offset = np.random.uniform(-0.0001, 0.0001)
                    y_offset = np.random.uniform(-0.0001, 0.0001)
                    self.ax.text(x + x_offset, y + y_offset, street_name, fontsize=8, ha='center', color='black', weight='bold')
                    displayed_names.add(street_name)

    def plot_bicycle(self, lat, lon, theta, alpha):
        #theta -> bicikli szoge,  alpha -> elso kerek szoge
        lb, lw = 0.002, 0.001  #tengelyek hosszai
        rear_x, rear_y = lon, lat
        front_x = rear_x + lb * np.cos(theta)
        front_y = rear_y + lb * np.sin(theta)
        wheel_x = front_x + lw * np.cos(alpha)
        wheel_y = front_y + lw * np.sin(alpha)
        self.ax.plot([rear_x, front_x], [rear_y, front_y], color="blue", linewidth=2)
        self.ax.plot([front_x, wheel_x], [front_y, wheel_y], color="red", linewidth=2)

    def show_map(self):
        plt.show()
