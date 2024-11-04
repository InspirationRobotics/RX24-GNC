'''
A simple mission template for creating new missions.

The variables and functions defined here must exist in all mission classes.
Comments are provided to explain the purpose of each variable and function and can be removed.
'''

from typing import Dict, Tuple

from comms_core import Logger
from ..mission_node import PositionData
from perception_core import CameraData, Results

import math



class NavMission(Logger):

    # Define the initial perception commands here (if any)
    # This dictionary must exist, but some commands are entered as examples
    init_perc_cmd = {}

    def __init__(self):
        super().__init__(str(self))
        self.count = 0
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
        heartbeat = []
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
                target_waypoint = self.destination_point(position_data.lat, position_data.lon, position_data.heading, 50)
                gnc_cmd["waypoint"] = target_waypoint
                self.count += 1

        return perc_cmd, gnc_cmd

    def end(self):
        '''
        Any cleanup code can be placed here.
        This function is called when the run function returns end_mission = True 
        or when the mission handler decides to end the mission.
        '''
        pass
    
    
    def destination_point(self, lat, lon, bearing, distance) -> Tuple[float, float]:
        """
        Calculate the destination point given a starting point, bearing, and distance in meters
        """
        # convert decimal degrees to radians
        lon, lat, bearing = map(math.radians, [lon, lat, bearing])

        distance = distance/1000

        # calculate the destination point
        lat2 = math.asin(math.sin(lat) * math.cos(distance/6371) + math.cos(lat) * math.sin(distance/6371) * math.cos(bearing))
        lon2 = lon + math.atan2(math.sin(bearing) * math.sin(distance/6371) * math.cos(lat), math.cos(distance/6371) - math.sin(lat) * math.sin(lat2))
        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)
        return lat2, lon2