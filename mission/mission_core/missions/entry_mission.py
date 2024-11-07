'''
A simple mission template for creating new missions.

The variables and functions defined here must exist in all mission classes.
Comments are provided to explain the purpose of each variable and function and can be removed.
'''
import cv2
import math
import random
from typing import Dict, Tuple
from comms_core import Logger
from ..mission_node import PositionData
from perception_core import CameraData, Results

'''
Entry logic:

# If entrance model works:
# - Run the entrance model on the center camera
# - Mark the two buoys, and center to get through them
#   - If one buoys is smaller than the other (ex left smaller than right then left is further)
#   - So then strafe left and yaw to the right
# - Once both buoys are out of frame, switch model to the platform model.

# Otherwise:
 - Go straight for a certain distance
 - Use the stc model to detect the platform and head towards it
 - Once we also get red/black/green/blue detections we can end mission and hand off to the next mission

Use new STC model to find the STC platform.
The model should run on all 3 cameras. If the center camera does not see the platform,
and one of the side cameras do, then yaw in the direction of the camera that sees the platform.
Head towards platform until its a certain size in the center camera.
End mission
'''


class EntryMission(Logger):

    # Define the initial perception commands here (if any)
    # This dictionary must exist, but some commands are entered as examples
    init_perc_cmd = {
        "start": ["center", "port", "starboard"],
        "load_model": [("center", "stcC.pt"), ("port", "stcC.pt"), ("starboard", "stcC.pt")],
        # "stop": ["port", "starboard"],
        # "record": ["center"],
    }

    def __init__(self, debug_mode: bool = False):
        super().__init__(str(self))
        self.debug_mode = debug_mode
        self.color_count = 0
        if self.debug_mode:
            cv2.namedWindow("view", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("view", 640, 480)
        self.at_target = False

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

        center_tower_location = self.process_tower_location("center", center_camera_data)
        if center_camera_data is not None:
            if center_camera_data.frame is not None and self.debug_mode:
                cv2.imshow("view", center_camera_data.frame)
                if cv2.waitKey(2) & 0xFF == ord('q'):
                    pass
        port_tower_location = self.process_tower_location("port", port_camera_data)
        starboard_tower_location = self.process_tower_location("starboard", starboard_camera_data)

        # If any camera sees a color, add to the count
        if center_tower_location is True or port_tower_location is True or starboard_tower_location is True:
            self.color_count += 1
            return perc_cmd, gnc_cmd

        # 5 frames of color detection is enough to assume we are at the platform
        if self.color_count >= 5:
            gnc_cmd = {
                "poshold": True,
                "end_mission": True
            }
            return perc_cmd, gnc_cmd

        gnc_cmd["heading"] = position_data.heading - 3
        # If the center camera sees the platform, head towards it
        if center_tower_location is not None:
            ratio, conf = center_tower_location
            if abs(ratio) < 0.1:
                gnc_cmd["vector"] = [0,0.5]
                gnc_cmd["heading"] = position_data.heading
            elif ratio < 0:
                gnc_cmd["heading"] = position_data.heading - 6
            elif ratio > 0:
                gnc_cmd["heading"] = position_data.heading + 6
            return perc_cmd, gnc_cmd
        
        # If the port camera sees the platform, yaw to the left
        if port_tower_location is not None:
            ratio, conf = port_tower_location
            gnc_cmd["vector"] = [0,0.2]
            gnc_cmd["heading"] = position_data.heading - 15
            return perc_cmd, gnc_cmd
        
        # If the starboard camera sees the platform, yaw to the right
        if starboard_tower_location is not None:
            ratio, conf = starboard_tower_location
            gnc_cmd["vector"] = [0,0.2]
            gnc_cmd["heading"] = position_data.heading + 15
            return perc_cmd, gnc_cmd

        perc_cmd = {}
        gnc_cmd = {}
        return perc_cmd, gnc_cmd

    def process_tower_location(self, name, camera_data: CameraData) -> Tuple[float, float] | None | bool:
        '''
        This function processes the tower location from the camera data.
        It returns the tower location in the form of a tuple (ratio, conf) where ratio is the distance of the tower
        from the center of the image.
        and conf is the confidence of the tower detection.
        If no tower is detected, it returns None.
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
                if conf > 0.3:
                    # Append the class id, confidence, and bounding box coordinates to conf_list
                    conf_list.append((int(cls_id), conf, (x1, y1, x2, y2)))
                        
        if len(conf_list) != 0:
            # If a color and platform is detected, return True
            for cls_id, conf, _ in conf_list:
                if cls_id in [0,1,2,3] and conf > 0.6:
                    if any([cls_id == 4 for cls_id, conf, _ in conf_list]):
                        self.warning("Color and platform detected")
                        return True and name == "center"
            # If a tower is detected, return the tower location
            for conf in conf_list:
                if conf[0] == 4:
                    x1, y1, x2, y2 = conf[2]
                    x = (x1 + x2) / 2
                    ratio = (x / frame_width) - 0.5
                    self.warning(f"{name}: Tower detected {ratio}, {conf[1]}")
                    return ratio, conf[1]
        # No detections
        return None
    
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
    
    def haversine(lat1, lon1, lat2, lon2) -> float:
        """
        Calculate the great circle distance between two points
        on the earth (specified in decimal degrees)
        Returns distance in meters
        """
        # convert decimal degrees to radians
        lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371000 # Radius of earth in meters. Use 3956 for miles
        return c * r

    def end(self):
        '''
        Any cleanup code can be placed here.
        This function is called when the run function returns end_mission = True 
        or when the mission handler decides to end the mission.
        '''
        pass