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
import cv2
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

    def __init__(self, targetColor=2, debug_mode=False):
        super().__init__(str(self))
        self.targetScanDockHeading = 180
        self.targetFindMissionHeading = 160
        self.count = 0
        self.closeToBackboard = False
        self.AMSStatus = 1      #1 is scanning, 2 is docking
        self.DeliveryStatus = 1
        self.targetColor = targetColor    # set from other function
        self.confThreshold = 0.5
        self.tolerance = 200  # Adjust tolerance as needed
        self.confThresholdBackboard = 0.35
        self.targetDockHeading = 270
        """
        0 red
        1 green
        2 blue
        3 backboard
        """
        self.debug_mode = debug_mode
        self.color_count = 0
        if self.debug_mode:
            cv2.namedWindow("view", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("view", 640, 480)

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
        
        
        self.count = 0
        if (not self.closeToBackboard):
            # We need to start getting closer to the backboard before scanning
            if self.count < 2 and position_data is not None:
                if position_data.heading is not None and position_data.position is not None:
                    self.log(f"Setting end waypoint to somewhere we can see the backboard")
                    target_waypoint = self.destination_point(position_data.lat, position_data.lon, self.targetFindMissionHeading, 30)
                    gnc_cmd["waypoint"] = target_waypoint
                    self.count += 1
            
            # At the end of here we need a condition for when we are close enough to stop, 
            # turn to target heading, and set a waypoint infront of us to navigate to while we scan
            # and set closeToBackboard to true
            perc_cmd = {}
            gnc_cmd = {}

            gnc_cmd["vector"] = [0, 0.3]

            if position_data is None:
                return perc_cmd, gnc_cmd
            
            if position_data.heading is None:
                return perc_cmd, gnc_cmd

            center_camera_data = camera_data.get("center")
            port_camera_data = camera_data.get("port")
            starboard_camera_data = camera_data.get("starboard")

            center_backboard_location = self.process_backboard_location("center", center_camera_data)
            if center_camera_data is not None:
                if center_camera_data.frame is not None and self.debug_mode:
                    cv2.imshow("view", center_camera_data.frame)
                    if cv2.waitKey(2) & 0xFF == ord('q'):
                        pass
            port_backboard_location = self.process_backboard_location("port", port_camera_data)
            starboard_backboard_location = self.process_backboard_location("starboard", starboard_camera_data)

            # If any camera sees a color, add to the count
            if center_backboard_location is True or port_backboard_location is True or starboard_backboard_location is True:
                self.color_count += 1
                return perc_cmd, gnc_cmd

            # 5 frames of color detection is enough to assume we are at the platform
            if self.color_count >= 5:
                gnc_cmd = {
                    "poshold": True,
                }
                self.warning("Got 5 color detections, going ")
                self.closeToBackboard = True
                self.count = 0
                return perc_cmd, gnc_cmd

            gnc_cmd["heading"] = position_data.heading - 3
            # If the center camera sees the platform, head towards it
            if center_backboard_location is not None:
                ratio, conf = center_backboard_location
                if abs(ratio) < 0.1:
                    gnc_cmd["vector"] = [0,0.5]
                    gnc_cmd["heading"] = position_data.heading
                elif ratio < 0:
                    gnc_cmd["heading"] = position_data.heading - 6
                elif ratio > 0:
                    gnc_cmd["heading"] = position_data.heading + 6
                return perc_cmd, gnc_cmd
            
            # If the port camera sees the platform, yaw to the left
            if port_backboard_location is not None:
                ratio, conf = port_backboard_location
                gnc_cmd["vector"] = [0,0.2]
                gnc_cmd["heading"] = position_data.heading - 15
                return perc_cmd, gnc_cmd
            
            # If the starboard camera sees the platform, yaw to the right
            if starboard_backboard_location is not None:
                ratio, conf = starboard_backboard_location
                gnc_cmd["vector"] = [0,0.2]
                gnc_cmd["heading"] = position_data.heading + 15
                return perc_cmd, gnc_cmd

            return perc_cmd, gnc_cmd
            
        
        else:
            # We are close to the back board 
            #TODO Question
            # Need to waypoint navigate to starting position
            
            # Need to start slowly navigating to end waypoint
            if self.count < 2 and position_data is not None:
                if position_data.heading is not None and position_data.position is not None:
                    self.log(f"Setting end waypoint to end of the dock")
                    # Just want to point at 180 degrees and start going along the dock
                    target_waypoint = self.destination_point(position_data.lat, position_data.lon, self.targetScanDockHeading, 30)

                    gnc_cmd["waypoint"] = self.endingWaypoint
                    self.count += 1
            
            
            # if self.count < 2 and position_data is not None:
            #     if position_data.heading is not None and position_data.position is not None:
            #         # This is to ensure we dont keep recalculating a different waypoint as we move
            #         target_waypoint = self.destination_point(position_data.lat, position_data.lon, position_data.heading, 50)
            #         gnc_cmd["waypoint"] = target_waypoint
            #         self.count += 1
            
            # Start scanning with starboard camera for the selected color
            if(self.AMSStatus == 1):
                cam_data = camera_data.get("starboard")
                cam_results = cam_data.results
                cam_frame = cam_data.frame
            elif (self.AMSStatus == 2):
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
            
            if self.AMSStatus == 1 and is_centered:
                # Moving to new stage where we turn and move forward     
                self.AMSStatus = 2
                #Turn 90 degrees  
                if position_data.heading is not None and position_data.position is not None:
                    target_waypoint = self.destination_point(position_data.lat, position_data.lon, self.targetDockHeading, 10)
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
    
    
    def process_backboard_location(self, name, camera_data: CameraData) -> Tuple[float, float] | None | bool:
        '''
        This function processes the backboard location from the camera data.
        It returns the backboard location in the form of a tuple (ratio, conf) where ratio is the distance of the backboard
        from the center of the image.
        and conf is the confidence of the backboard detection.
        If no backboard is detected, it returns None.
        If it starts detecting the colors, it returns True.
        '''
        results = camera_data.results
        if camera_data.results is None:
            return None
        if camera_data.frame is None:
            return None
        
        frame_width = camera_data.frame.shape[1]

        conf_list = []
        for result in results:
            for box in result.boxes:
                conf = box.conf.item()
                cls_id = box.cls.item()
                x1, y1, x2, y2 = box.xyxy.cpu().numpy().flatten()
                if conf > self.confThresholdBackboard:
                    # Append the class id, confidence, and bounding box coordinates to conf_list
                    conf_list.append((int(cls_id), conf, (x1, y1, x2, y2)))
                        
        if len(conf_list) != 0:
            # If a color and backboard is detected, return True
            for cls_id, conf, _ in conf_list:
                if cls_id in [0,1,2,3] and conf > 0.6:
                    if any([cls_id == 4 for cls_id, conf, _ in conf_list]):
                        self.warning("Color and backboard detected")
                        return True and name == "center"
            # If a backboard is detected, return the backboard location
            for conf in conf_list:
                if conf[0] == 4:
                    x1, y1, x2, y2 = conf[2]
                    x = (x1 + x2) / 2
                    ratio = (x / frame_width) - 0.5
                    self.warning(f"{name}: backboard detected {ratio}, {conf[1]}")
                    return ratio, conf[1]
        # No detections
        return None