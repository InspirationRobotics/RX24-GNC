'''
A simple mission template for creating new missions.

The variables and functions defined here must exist in all mission classes.
Comments are provided to explain the purpose of each variable and function and can be removed.
'''

from typing import Dict, Tuple
import random
from comms_core import Logger
from ..mission_node import PositionData
from perception_core import CameraData, Results


class EntryMission(Logger):

    # Define the initial perception commands here (if any)
    # This dictionary must exist, but some commands are entered as examples
    init_perc_cmd = {
        "start": ["center", "port", "starboard"],
        "load_model": [("center", "stcA.pt")],
        # "stop": ["port", "starboard"],
        # "record": ["center"],
    }

    def __init__(self):
        super().__init__(str(self))
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
        heartbeat = ["RXCOD", self.light_pattern]
        '''
        heartbeat = ["RXGAT", random.choice(["1", "2", "3"]), random.choice(["1", "2", "3"])]
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
            "vector": tuple, # The target vector for the boat to head towards (will override heading #TODO: TEST THIS)
            "poshold": bool, # Whether to set the GNC to poshold mode (heading is not maintained but position is)
            "waypoint": tuple, # The target waypoint to set the GNC to (This can also be a list of waypoints) (sending any other commands will override this)
            "end_mission": bool # Whether to end the mission
        }

        You can log/print data by using the self.log() function inherited from the Logger class.
        It is f-string compatible so you can call it like so:
            self.log(f"Position: {position_data.position}")
            or just self.log("hi there") or self.log(variable)

        '''

        # The logic for this mission is essentially to move forward toward the STC until the side cameras no longer see the buoys
        # This involves moving forward and yawing at the same time
        # The STC is the center camera

        center_camera_data = camera_data.get("center")
        center_camera_results = center_camera_data.results
        if center_camera_results is not None:
            frame_width, frame_height = center_camera_data.frame.shape[1], center_camera_data.frame.shape[0]
            conf_list = []
            for result in center_camera_results:
                for box in result.boxes:
                    conf = box.conf.item()
                    cls_id = box.cls.item()
                    x1, y1, x2, y2 = box.xyxy.cpu().numpy().flatten()
                    
                    # Append the class id, confidence, and bounding box coordinates to conf_list
                    conf_list.append((int(cls_id), conf, (x1, y1, x2, y2)))
                            
            if len(conf_list) != 0:
                highest_confidence_result = max(conf_list, key=lambda x: x[1])
                bounding_box_coords = highest_confidence_result[2]
                x1, y1, x2, y2 = bounding_box_coords
                center_coord = (x1 + x2) / 2, (y1 + y2) / 2
                center_x, center_y = center_coord
                center_x = center_x / frame_width
                if center_x < 0.45:
                    print("The STC is to the left")
                elif center_x > 0.55:
                    print("The STC is to the right")
                else:
                    print("The STC is centered")
        
        gnc_cmd = {}
        # So to test:
        if position_data is not None:
            if position_data.heading is not None:
                target_heading = position_data.heading + 5
                target_vector = [0,0.6] # Move forward at 0.5 speed
                gnc_cmd["heading"] = target_heading
                gnc_cmd["vector"] = target_vector

        perc_cmd = {}
        return perc_cmd, gnc_cmd

    def end(self):
        '''
        Any cleanup code can be placed here.
        This function is called when the run function returns end_mission = True 
        or when the mission handler decides to end the mission.
        '''
        pass