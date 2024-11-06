'''
A simple mission template for creating new missions.

The variables and functions defined here must exist in all mission classes.
Comments are provided to explain the purpose of each variable and function and can be removed.
'''

from typing import Dict, Tuple

from comms_core import Logger
from ..mission_node import PositionData
from perception_core import CameraData, Results


class FollowPatj(Logger):

    # Define the initial perception commands here (if any)
    # This dictionary must exist, but some commands are entered as examples
    init_perc_cmd = {
        "start": ["center"],
        "stop": ["port", "starboard"],
        # "record": ["center"],
        "load_model": [("center", "stcA.pt"), ("starboard", "stcA.pt")], # need to adjust to follow the path

    }

    def __init__(self):
        super().__init__(str(self))
        self.status = 1 # in progress 2 completed
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
        heartbeat = ["$RXPTH", self.status]
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
        
        # Run one yolo inference on the provided frame at camera_data.frame
        center_camera_data = camera_data.get("starboard")
        center_camera_results = center_camera_data.results

        if center_camera_results is None:
            return perc_cmd, gnc_cmd
            
        # Find the result with the highest confidence
        highest_confidence_result = max(conf_list, key=lambda x: x[1])
        
        # Extract the class ID, confidence, and bounding box coordinates (maybe this corresponds to the largest bouy)
        bounding_box_class_id = highest_confidence_result[0]
        bounding_box_coords = highest_confidence_result[2]
        
        x1, y1, x2, y2 = bounding_box_coords
        
        average_x = abs(x1 - x2)/2
        
        limit_x_l = 10 # lower bound (pixels)
        limit_x_u = 200 # upper bound (pixels)
        
        if average_x < limit_x_l: 
            # forward
        elif average_x > limit_x_u:
            # forwardd
        else: 
            if bounding_box_class_id = "red" :
                # red (assume that red is on the port side)
                # yaw starboard (turning to force the bouy onto the edge)
                
            elif bounding_box_class_id = "green"  :
                # green (assume that green is on the starboard side)
                # yaw port (tunring to fouce to the bouy onto the edge)
            else: 
                # black bouy (not sure how to handle) 
                # red blue bouy (entrerance if red is on starboard) (initialize the program) (defualt)
                # red green bouy ( exit if red is on starboard) (terminate the program) 
                
            
            
        return perc_cmd, gnc_cmd

        
        '''
        if on the left side of the frame, forward
        if on the right side of the frame, forward
        if green 
        '''


    def end(self):
        '''
        Any cleanup code can be placed here.
        This function is called when the run function returns end_mission = True 
        or when the mission handler decides to end the mission.
        '''
        pass
        
        # 0 is black
        # 1 is blue
        # 2 is green
        # 3 is red