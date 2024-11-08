'''
A simple mission template for creating new missions.

The variables and functions defined here must exist in all mission classes.
Comments are provided to explain the purpose of each variable and function and can be removed.
'''

from typing import Dict, Tuple
import time
import cv2
from comms_core import Logger
from ..mission_node import PositionData
from ..GIS import haversine, destination_point
from perception_core import CameraData, Results


class FTPMission(Logger):

    # Define the initial perception commands here (if any)
    # This dictionary must exist, but some commands are entered as examples
    init_perc_cmd = {
        "start": ["center"],
        "stop": ["port", "starboard"],
        "load_model": [("center", "FTPB.pt")]
    }

    def __init__(self, left_color, start_waypoint : Tuple[float, float] = None, start_heading : float = None, debug_mode: bool = False):
        super().__init__(str(self))
        if left_color != "R" and left_color != "G":
            raise ValueError("left_color must be either 'R' or 'G'")
        self.left_color = left_color

        self.start_waypoint = start_waypoint
        self.at_target = False
        self.start_heading = start_heading
        self.at_heading = False

        self.end_waypoint = None

        self.active = False
        self.last_detection = time.time()

        self.debug_mode = debug_mode
        if self.debug_mode:
            cv2.namedWindow("view", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("view", 640, 480)

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
        heartbeat = ["RXPTH", "R", "1"]
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

        # FTP logic
        '''
        We have a model loaded in the center camera that can detect the FTP buoy.
        Left color will either be R or G.
        The model will return all detections of either Red, Green, or Black buoys.
        We will select the buoy that is the largest bounding box detection.
         - If a black buoy is detected we will use which half of the screen it is on 
            to determine if it should be on the right or left. This will be the priority.
        If no black buoy is detected we will use the color of the buoy to determine if it is the left or right buoy.
        If the buoy is a left buoy we will yaw until the buoy is on the left 5-15% of the screen. 
            If its less than 15% we will stop yawing and move forward. If its less than 5% we will stop and yaw right.
        If the buoy is a right buoy we will yaw until the buoy is on the right 5-15% of the screen.
            If its less than 15% we will stop yawing and move forward. If its less than 5% we will stop and yaw left.
        
        The mission ends if there are no more detections for 5 seconds.
        '''

        perc_cmd = {}
        gnc_cmd = {}

        gnc_cmd["vector"] = [0, 0]

        if position_data is None:
            return perc_cmd, gnc_cmd
        
        if position_data.heading is None:
            return perc_cmd, gnc_cmd
        
        if self.start_waypoint is not None and self.start_heading is not None:
            # Check if we are at the start waypoint and heading
            if haversine(position_data.lat, position_data.lon, self.start_waypoint[0], self.start_waypoint[1]) > 3 and not self.at_target:
                gnc_cmd["waypoint"] = self.start_waypoint
                self.warning(f"Moving towards waypoint: {self.start_waypoint}")
                return perc_cmd, gnc_cmd
            self.at_target = True

            if abs(position_data.heading - self.start_heading) > 5 and not self.at_heading:
                gnc_cmd["heading"] = self.start_heading
                self.warning(f"Turning towards heading: {self.start_heading}")
                return perc_cmd, gnc_cmd
            self.at_heading = True

        # See if we are in end mission state
        if self.end_waypoint is not None:
            self.warning(f"Heading towards waypoint: {self.end_waypoint}")
            if haversine(position_data.lat, position_data.lon, self.end_waypoint[0], self.end_waypoint[1]) < 2:
                gnc_cmd["end_mission"] = True
                return perc_cmd, gnc_cmd

        # Begin FTP logic
        center_camera_data = camera_data.get("center")
        data = self.process_buoy(center_camera_data)
        if data is None:
            if time.time() - self.last_detection > 5 and self.active and self.end_waypoint is None:
                self.end_waypoint = destination_point(position_data.lat, position_data.lon, position_data.heading, 8) # move 5 meters forward
                gnc_cmd["waypoint"] = self.end_waypoint
            return perc_cmd, gnc_cmd
        if not self.active:
            self.active = True
        self.last_detection = time.time()

        location, ratio = data
        target_speed, yaw_rate = self.navigate(location, ratio)
        gnc_cmd["vector"] = [0, target_speed]
        gnc_cmd["heading"] = position_data.heading + yaw_rate

        return perc_cmd, gnc_cmd
    
    def navigate(self, location : str, ratio : float) -> Tuple[float, float]:
        '''
        Based off the proposed location and current location return the forward speed and the yaw rate.
        '''
        target_speed = 0.2
        yaw_rate = 0
        # If the buoy should be on the left side of the screen.
        if location == "L":
            if ratio < 0.05:
                yaw_rate = -6
            elif ratio < 0.15:
                target_speed = 0.3
            else:
                yaw_rate = 6
        else:
            if ratio > 0.95:
                yaw_rate = 6
            elif ratio > 0.85:
                target_speed = 0.3
            else:
                yaw_rate = -6
        # yaw_rate*=2
        if yaw_rate > 0:
            self.warning(f"Yawing right at {yaw_rate}")
        elif yaw_rate < 0:
            self.warning(f"Yawing left at {yaw_rate}")
        else:
            self.warning(f"Moving forward at {target_speed}")
        return target_speed, yaw_rate

    def process_buoy(self, camera_data: CameraData) -> Tuple[str, float] | None:
        '''
        Returns None if no valid detections.
        Returns a tuple of the proposed buoy location and the location of the buoy on the screen as a ratio.
        0 is the left of the frame, 1 is the rightmost part of the frame.
        '''

        if camera_data is None:
            return None
        results = camera_data.results
        if camera_data.results is None:
            return None
        if camera_data.frame is None:
            return None
        
        if self.debug_mode:
            cv2.imshow("view", camera_data.frame)
            if cv2.waitKey(2) & 0xFF == ord('q'):
                pass

        frame_width = camera_data.frame.shape[1]

        conf_list = []
        for result in results:
            for box in result.boxes:
                conf = box.conf.item()
                cls_id = box.cls.item()
                x1, y1, x2, y2 = box.xyxy.cpu().numpy().flatten()
                if conf > 0.55:
                    # Append the class id, confidence, and bounding box coordinates to conf_list
                    size = (x2 - x1) * (y2 - y1)
                    conf_list.append((int(cls_id), conf, (x1, y1, x2, y2), size))

        if len(conf_list) == 0:
            return None
        
        # Sort the conf_list by the size of the bounding box
        conf_list = sorted(conf_list, key=lambda x: x[3], reverse=True)

        # Check for a black buoy (id = 2)
        for conf in conf_list:
            if conf[0] == 2:
                target_buoy = conf
                break
            else: # If no black buoy is detected, use the largest bounding box
                target_buoy = conf_list[0]

        # Ensure its either red, green, or black
        if target_buoy[0] not in [0, 1, 2]:
            return None
        
        lookup = {0: "Red", 1: "Green", 2: "Black"}

        # Determine where it is on screen
        center_x = (target_buoy[2][0] + target_buoy[2][2]) / 2
        ratio = center_x / frame_width

        self.warning(f"Buoy detected: {lookup[target_buoy[0]]} with ratio: {ratio}")

        # Determine where the target buoy should be:
        if target_buoy[0] == 2:
            if ratio > 0.5:
                location = "R"
            else:
                location = "L"
            return location, ratio
        else: # cls_id 0 is red, cls_id 1 is green
            if target_buoy[0] == 0 and self.left_color == "R":
                return "L", ratio
            elif target_buoy[0] == 1 and self.left_color == "G":
                return "L", ratio
            else:
                return "R", ratio


    def end(self):
        '''
        Any cleanup code can be placed here.
        This function is called when the run function returns end_mission = True 
        or when the mission handler decides to end the mission.
        '''
        pass