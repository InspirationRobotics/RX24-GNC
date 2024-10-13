import time
import rclpy
from threading import Thread, Lock
from typing import Dict, List, Any

from comms_core import Server, Logger, CustomSocketMessage as csm
from perception_core import Perception, CameraData, Results

from mission_core.mission_node import MissionNode
from missions.mission_template import SimpleMission

'''
The concept of this class is simple.
It is the main class that will handle the mission.

It will have a callback function which will run at a fixed rate.
The callback function will take in a dictionary with the camera data organized like so:
{
    "port": CameraData
    "starboard": CameraData
    "center": CameraData
}
Where CameraData is a class that contains the image frame and the results of a Yolo model (if running).

The callback function will also take in a PositionData object which contains the current position and heading of the vehicle.
The callback function will also take in the latest occupancy grid data. (format TBD)

The callback function will output a dictionary with the following data:
{
    per_cmd: {} # Dictionary containing the commands for the perception module (defined in perception_core)
    gnc_cmd: {} # Dictionary containing the commands for the GNC module
}
gnc_cmd will have:
{
    "heading": float # The target heading to set the GNC to (will auto switch to Mode 0)
    "vector": tuple # The target vector to set the GNC to (will auto switch to Mode 0)
    "poshold": bool # Whether to set the GNC to poshold mode (switch to mode 1)
    "waypoint": tuple # The target waypoint to set the GNC to (will auto switch to Mode 2)
    "end_mission": bool # Whether to end the mission
}

While the mission_handler is active, it will be sending a heartbeat to the LLC module at 5hz along with any of the GNC commands.

'''

class PositionData:
    def __init__(self, position: tuple, heading: float):
        self.position = position
        self.lat = position[0] if position is not None else None
        self.lon = position[1] if position is not None else None
        self.heading = heading

class MissionHandler(Logger):
    
    def __init__(self, mission_list: List[SimpleMission] = None):
        super().__init__("MissionHandler")
        self.mission_list = mission_list if mission_list is not None else []
        self.current_mission : SimpleMission = None

        self.server = Server(default_callback=self._server_callback)
        self.perception = Perception()

        rclpy.init(args=None)
        self.mission_node = MissionNode()

        self.position_data : PositionData = None

        self.active = False

        self.callback_thread = Thread(target=self.__callback_loop)
        self.callback_active = False

        self.send_thread = Thread(target=self.__send_loop)
        self.send_lock = Lock()
        self.to_send = {}
        
        self.log("Mission Handler Initialized.")
        self.start()

    def _server_callback(self, data, addr):
        data = csm.decode(data, as_interface=True)
        self.position_data = PositionData(data.position, data.heading)
        self.mission_node.send_gps(self.position_data)

    def __parse_gnc_cmd(self, gnc_cmd: Dict[str, Any]):
        with self.send_lock:
            if "heading" in gnc_cmd:
                self.to_send["set_mode"] = 0
                self.to_send["target_heading"] = gnc_cmd["heading"]
            if "vector" in gnc_cmd:
                self.to_send["set_mode"] = 0
                self.to_send["target_vector"] = gnc_cmd["vector"]
            if "poshold" in gnc_cmd:
                if gnc_cmd["poshold"]:
                    self.to_send["set_mode"] = 1
            if "waypoint" in gnc_cmd:
                self.to_send["set_mode"] = 2
                self.to_send["target_position"] = gnc_cmd["waypoint"]
            if "end_mission" in gnc_cmd:
                self.to_send["set_mode"] = 0
                return gnc_cmd["end_mission"]
        return False

    def __callback_loop(self):
        while self.active:
            while self.callback_active:
                per_cmd, gnc_cmd = self.current_mission.run(self.perception.get_latest_data(), self.position_data)
                end_mission = self.__parse_gnc_cmd(gnc_cmd)
                self.perception.command_perception(per_cmd)
                if end_mission:
                    self.next_mission()
                time.sleep(1/20)
            time.sleep(0.5)

    def __send_loop(self):
        while self.active:
            with self.send_lock:
                self.to_send["heartbeat"] = time.time()
                self.server.send(csm.encode(self.to_send), "192.168.3.6")
                self.to_send = {}
            time.sleep(0.2)

    def start(self):
        self.log("Starting Mission Handler.")
        self.active = True
        self.mission_node.start()
        self.callback_thread.start()
        self.send_thread.start()

    def stop(self):
        self.log("Stopping Mission Handler.")
        self.active = False
        self.callback_thread.join()
        self.send_thread.join()
        self.mission_node.stop()
        rclpy.shutdown()

    def start_mission(self):
        self.log("Starting Missions.")
        self.next_mission()

    def pause_mission(self):
        self.log("Pausing Missions.")
        self.callback_active = False

    def resume_mission(self):
        self.log("Resuming Missions.")
        self.callback_active = True

    def next_mission(self):
        self.callback_active = False
        if self.current_mission is not None:
            self.log(f"Ending current mission: {self.current_mission}")
            self.current_mission.end()
        if len(self.mission_list) == 0:
            self.log("No more missions to run.")
            self.stop()
            return
        self.current_mission = self.mission_list.pop(0)
        self.log(f"Starting next mission: {self.current_mission}")
        self.perception.command_perception(self.current_mission.init_perc_cmd)
        self.callback_active = True

    def load_mission(self, mission: SimpleMission):
        self.log(f"Added mission: {mission}")
        self.mission_list.append(mission)