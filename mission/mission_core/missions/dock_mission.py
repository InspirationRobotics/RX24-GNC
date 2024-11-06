'''
Dock Mission

The variables and functions defined here must exist in all mission classes.
Comments are provided to explain the purpose of each variable and function and can be removed.
'''

from typing import Dict, Tuple

from comms_core import Logger
from ..mission_node import PositionData
from perception_core import CameraData, Results

import math
import time


class DockMission(Logger):

    # Define the initial perception commands here (if any)
    # This dictionary must exist, but some commands are entered as examples
    init_perc_cmd = {
        "start": ["center", "starboard"],
        "stop": ["port"],
        # "record": ["center"],
        "load_model": [("center", "dock.pt"), ("starboard", "dock.pt")],
    }

    def __init__(self):
        super().__init__(str(self))
        self.startingWaypoint = (0,0)
        self.endingWaypoint = (0,0)
        self.count = 0
        self.AMSStatus = 1
        self.DeliveryStatus = "S"
        self.targetColor = 2    # TODO set from other function
        self.confThreshold = 0.5
        self.tolerance = 100  # Adjust tolerance as needed

        self.log(f"Starting Dock Mission")

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
        
        
        # For self.AMSStatus, 1 for docking, 2 for complete
        # For self.DeliveryStatus, S for scanning, D for delivering
        
        heartbeat = ["$RXDOK", self.AMSStatus, self.DeliveryStatus]
        return heartbeat

    def run(self, camera_data: Dict[str, CameraData], position_data: PositionData, occupancy_grid = None) -> Tuple[Dict, Dict, Dict]:
        '''
        Camera data is a dictionary with the following format:
        {
            "port": CameraData,
            "starboard": CameraData,
            "center": CameraData
        }
        CameraData is a class that contains the image frame and the results of a Yolo model (if running).
        And can be accessed like so:
            camera_data.frame
            camera_data.results

        PositionData is a class that contains the current position and heading of the vehicle.
        And can be accessed like so:
            position_data.lat : float
            position_data.lon : float
            position_data.position : tuple (lat, lon)
            position_data.heading : float

        The output perc_cmd has the following format (does not have to be filled):
        You only need to put any arguements in here if you want to change the current perception config.
        {
            "start": ["camera_name"],
            "stop": ["camera_name"],
            "record": ["camera_name"],
            "stop_record": ["camera_name"],
            "load_model": [("camera_name", "model_path")],
            "stop_model": ["camera_name"]
        }

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
        
        
        #TODO Question
        # Need to waypoint navigate to starting position
        
        # Need to start slowly navigating to end waypoint
        if self.count < 2 and position_data is not None:
            if position_data.heading is not None and position_data.position is not None:
                self.log(f"Setting end waypoint to end of the dock")
                gnc_cmd["waypoint"] = self.endingWaypoint
                self.count += 1
        
        
        # if self.count < 2 and position_data is not None:
        #     if position_data.heading is not None and position_data.position is not None:
        #         # This is to ensure we dont keep recalculating a different waypoint as we move
        #         target_waypoint = self.destination_point(position_data.lat, position_data.lon, position_data.heading, 50)
        #         gnc_cmd["waypoint"] = target_waypoint
        #         self.count += 1
        
        # Start scanning with starboard camera for the selected color
        if(self.AMSStatus == "S"):
            cam_data = camera_data.get("starboard")
            cam_results = cam_data.results
            cam_frame = cam_data.frame
        elif (self.AMSStatus == "D"):
            cam_data = camera_data.get("center")
            cam_results = cam_data.results
            cam_frame = cam_data.frame

        if cam_results is None:
            return perc_cmd, gnc_cmd
        
        detectedColors = {}
        for result in cam_results:
            for box in result.boxes:
                conf = box.conf.item()
                cls_id = box.cls.item()
                x1, y1, x2, y2 = box.xyxy.cpu().numpy().flatten()
                if (conf >= self.confThreshold):
                    # Append the class id, confidence, and bounding box coordinates to list
                    
                    detectedColors[cls_id] = ((x1, y1, x2, y2))
                        
        if len(detectedColors) == 0:
            return perc_cmd, gnc_cmd
        
        self.log(f"Current AMSStatus: {self.AMSStatus}, Current DeliveryStatus: {self.DeliveryStatus}, Detected colors: {detectedColors}")
        # Check and see if we found the desired color:
        bounding_box_coords = detectedColors[self.targetColor]
        if(bounding_box_coords is None):
            # No color found so skip
            return perc_cmd, gnc_cmd
            
            
        # Extract the bounding box coordinates        
        x1, y1, x2, y2 = bounding_box_coords
        _, width = cam_frame.shape[:2]
        frame_center_x = width // 2


        # Calculate the center of the bounding box
        box_center_x = (x1 + x2) // 2

        # Check if the bounding box center is at the frame center
        
        is_centered = (
            abs(box_center_x - frame_center_x) <= self.tolerance
        )
        
        self.log(f"isCentered: {is_centered}, box_center: {box_center_x}, frame_center: {frame_center_x}, difference: {box_center_x - frame_center_x}, Tolerence: {self.tolerence}")
        
        if self.AMSStatus == "S" and is_centered:
            # Moving to new stage where we turn and move forward     
            self.AMSStatus = "D"
            #Turn 90 degrees  
            if position_data.heading is not None and position_data.position is not None:
                target_waypoint = self.destination_point(position_data.lat, position_data.lon, position_data.heading + 90, 10)
                start_time = time.time()

                gnc_cmd["waypoint"] = target_waypoint
                return perc_cmd, gnc_cmd
    
        # Fire racket ball#TODO
        self.log(f"Would send command to fire racketball")
        
        # End mission #TODO is this when we end mission
        if (self.AMSStatus == "D"):
                elapsed_time = time.time() - start_time
                self.log(f"Pulling into the dock")

                if elapsed_time >= 5:
                    self.log(f"In dock, ending mission")
                    gnc_cmd = {"end_mission": True} 
        
    
        
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