import time
import numpy as np
from threading import Thread, Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from mission_core.mission import PositionData
from interfaces.msg import LatLonHead, Occupancy, Grid, Cell

'''
A simple ROS node which publishes lat, lon, and heading data to the occupancy node every time a method is called.
It also subscribes to the Occupancy message and updates an accessable variable with the latest occupancy grid data.
'''

class OccupancyData:
    def __init__(self, msg: Occupancy):
        # Occupancy data
        self.timestamp = msg.header.stamp.sec
        self.origin = (msg.origin_latitude, msg.origin_longitude)
        self.cell_size = msg.cell_size
        # Grid data
        grid : Grid = msg.grid
        self.x_range = grid.x_range
        self.y_range = grid.y_range
        self.val_range = grid.value_range
        self.val_zero_point = grid.value_zero_point
        # Cells
        self.cells = {}
        cell : Cell
        for cell in grid.cells:
            self.cells[(cell.x_coord, cell.y_coord)] = cell.value


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')
        self.occupancy = None
        self._init_pubsubs()
        self.spin_thread = Thread(target=self._run, daemon=True)
        self.active = False

    def _init_pubsubs(self):
        sub_group = MutuallyExclusiveCallbackGroup()
        pub_group = MutuallyExclusiveCallbackGroup()
        self.occupancy_sub = self.create_subscription(Occupancy, '/RX/occupancy', self._occupancy_callback, 10, callback_group=sub_group)
        self.latlonhead_pub = self.create_publisher(LatLonHead, '/RX/latlonhead', 10, callback_group=pub_group)

    def _occupancy_callback(self, msg: Occupancy):
        self.occupancy = OccupancyData(msg)

    def get_occupancy(self):
        return self.occupancy

    def start(self):
        if self.active:
            return
        self.lidar_executor = MultiThreadedExecutor()
        self.lidar_executor.add_node(self)
        self.active = True
        self.spin_thread.start()
        print("Started mission node...")

    def send_gps(self, data: PositionData):
        msg = LatLonHead()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude = data.lat
        msg.longitude = data.lon
        msg.heading = data.heading
        self.latlonhead_pub.publish(msg)

    def stop(self):
        if not self.active:
            return
        self.active = False
        self.spin_thread.join(2)
        self.lidar_executor.shutdown()
        print("Stopped mission node...")

    def _run(self):
        while self.active:
            self.lidar_executor.spin_once()
            time.sleep(1/20)