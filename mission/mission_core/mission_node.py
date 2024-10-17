import time
import numpy as np
from typing import Dict, Tuple
from threading import Thread, Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from interfaces.msg import LatLonHead, Occupancy, Grid, Cell

'''
A simple ROS node which publishes lat, lon, and heading data to the occupancy node every time a method is called.
It also subscribes to the Occupancy message and updates an accessable variable with the latest occupancy grid data.
'''

class PositionData:
    def __init__(self, position: tuple, heading: float):
        self.position = position
        self.lat = position[0] if position is not None else None
        self.lon = position[1] if position is not None else None
        self.heading = heading


class OccupancyData:
    def __init__(self, msg: Occupancy):
        # Occupancy data
        self.timestamp = msg.header.stamp.sec
        self.origin = msg.origin
        self.position = msg.position[0:2]
        self.heading = msg.position[2]
        self.cell_size = msg.cell_size
        # Grid data
        grid : Grid = msg.grid
        self.x_range = grid.x_range
        self.y_range = grid.y_range
        self.val_range = grid.value_range
        self.val_zero_point = grid.value_zero_point
        # Cells
        self.cells : Dict[Tuple[int, int], int] = {}
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
        self.occupancy_sub = self.create_subscription(Occupancy, '/RX/occupancy_grid', self._occupancy_callback, 10, callback_group=sub_group)
        self.latlonhead_pub = self.create_publisher(LatLonHead, '/RX/gps', 10, callback_group=pub_group)

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
        if data.position is None:
            return
        if data.heading is None:
            return
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

if __name__ == '__main__':
    import cv2

    def global_to_local(lat: float, lon: float, origin: tuple, cell_size: float):
        x = (lat-origin[0]) * 111139
        y = (lon-origin[1]) * 111139 * np.cos(np.radians(lat))
        return int(x/cell_size), int(y/cell_size)

    def visualize_occupancy(occupancy: OccupancyData):
        if occupancy is None:
            return
        x_size = occupancy.x_range[1] - occupancy.x_range[0] + 1 + 60
        y_size = occupancy.y_range[1] - occupancy.y_range[0] + 1 + 60
        frame = np.zeros((y_size, x_size))

        def translate(coord):
            x = coord[0] - occupancy.x_range[0] + 30
            y = y_size - (coord[1] - occupancy.y_range[0] + 30)
            return x, y

        for coord in occupancy.cells:
            if occupancy.cells[coord] <= occupancy.val_zero_point:
                continue
            x, y = translate(coord)
            frame[y, x] = occupancy.cells[coord] / occupancy.val_range[1]

        boat_pos = global_to_local(occupancy.position[0], occupancy.position[1], occupancy.origin, occupancy.cell_size)
        boat_pos = translate(boat_pos)
        frame = np.array(frame * 255, dtype=np.uint8)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        cv2.circle(frame, boat_pos, 5, (0, 255, 0), -1)
        # Draw an arrow representing the heading
        heading = np.radians(occupancy.heading)
        x1 = boat_pos[0]
        y1 = boat_pos[1]
        x2 = x1 + int(20 * np.sin(heading))
        y2 = y1 - int(20 * np.cos(heading))
        cv2.arrowedLine(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        # frame = cv2.resize(frame, (800,800), interpolation=cv2.INTER_NEAREST)
        cv2.imshow("Occupancy Grid", frame)
        cv2.waitKey(1) & 0xFF

    # Test the MissionNode with occupancy grid visualization
    rclpy.init()
    mission_node = MissionNode()
    mission_node.start()
    while True:
        try:
            # mission_node.send_gps(PositionData((32.946099210867345, -117.13801643179173), 0.0))
            occupancy = mission_node.get_occupancy()
            visualize_occupancy(occupancy)
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    
    cv2.destroyAllWindows()
    mission_node.stop()
    rclpy.shutdown()