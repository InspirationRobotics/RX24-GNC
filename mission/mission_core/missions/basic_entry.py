'''
A simple mission template for creating new missions.

The variables and functions defined here must exist in all mission classes.
Comments are provided to explain the purpose of each variable and function and can be removed.
'''

from typing import Dict, Tuple

import random
from comms_core import Logger
from ..mission_node import PositionData
from ..GIS import haversine, destination_point
from perception_core import CameraData, Results

class BasicEntry(Logger):

    # Define the initial perception commands here (if any)
    # This dictionary must exist, but some commands are entered as examples
    init_perc_cmd = {}

    def __init__(self, distance=10):
        super().__init__(str(self))
        self.count = 0
        self.distance = distance
        self.target_waypoint = None
        pass

    def __str__(self):
        return self.__class__.__name__
    
    def mission_heartbeat(self):
        '''
        The heartbeat format can be found in the Team Handbook V3 Appendix D (Page 55).
        The heartbeat must specify all relevant information about the mission BESIDES the following:
         - Date
         - Time
         - Team ID
         - Checksum
        It must return a list of the required information in the correct order.
        For example, Scan the Code would be: 
        heartbeat = ["$RXCOD", self.light_pattern]
        '''
        heartbeat = ["RXGAT", random.choice(["1", "2", "3"]), random.choice(["1", "2", "3"])]
        return heartbeat

    def run(self, camera_data: Dict[str, CameraData], position_data: PositionData, occupancy_grid = None) -> Tuple[Dict, Dict, Dict]:
        '''
        PositionData is a class that contains the current position and heading of the vehicle.
        And can be accessed like so:
            position_data.lat : float
            position_data.lon : float
            position_data.position : tuple (lat, lon)
            position_data.heading : float


        The output gnc_cmd has the following format (does not have to be filled):
        {
            "heading": float, # The target heading for the boat to turn towards
            "vector": tuple, # The target vector for the boat to head towards (will override heading)
            "poshold": bool, # Whether to set the GNC to poshold mode (heading is not maintained but position is)
            "waypoint": tuple, # The target waypoint to set the GNC to (This can also be a list of waypoints) (sending any other commands will override this)
            "end_mission": bool # Whether to end the mission
        }

        You can log/print data by using the self.log() function inherited from the Logger class.
        It is f-string compatible so you can call it like so:
            self.log(f"Position: {position_data.position}")
            or just self.log("hi there") or self.log(variable)

        '''
        perc_cmd = {}
        gnc_cmd = {}
        if self.count < 2 and position_data is not None:
            if position_data.heading is not None and position_data.position is not None:
                # This is to ensure we dont keep recalculating a different waypoint as we move
                self.target_waypoint = destination_point(position_data.lat, position_data.lon, position_data.heading, self.distance)
                gnc_cmd["waypoint"] = self.target_waypoint
                self.count += 1

        if self.target_waypoint is not None and position_data is not None:
            distance = haversine(position_data.lat, position_data.lon, self.target_waypoint[0], self.target_waypoint[1])
            if distance < 2:
                gnc_cmd["end_mission"] = True

        return perc_cmd, gnc_cmd

    def end(self):
        '''
        Any cleanup code can be placed here.
        This function is called when the run function returns end_mission = True 
        or when the mission handler decides to end the mission.
        '''
        pass
    
    
    