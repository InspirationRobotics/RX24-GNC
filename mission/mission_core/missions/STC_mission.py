'''
A simple mission template for creating new missions.

The variables and functions defined here must exist in all mission classes.
Comments are provided to explain the purpose of each variable and function and can be removed.
'''

from typing import Dict, Tuple
from comms_core import Logger
from ..mission_node import PositionData
from perception_core import CameraData, Results
from itertools import groupby
import cv2
import numpy as np


# New data.yaml is as following : names: ['1red', '2green', '3blue', '4black', '5platform']

class STCMission(Logger):

    # Define the initial perception commands here (if any)
    # This dictionary must exist, but some commands are entered as examples
    init_perc_cmd = {
        "start": ["center", "starboard", "port"],
        # "record": ["center"],
        # "stop_record": ["port", "starboard"],
        "load_model": [("center", "stcA.pt"), ("starboard", "stcA.pt"), ("port", "stcA.pt")]
    }

    def __init__(self):
        super().__init__(str(self))
        self.records = {"123":0, "132":0, "213":0, "231":0, "312":0, "321":0 }
        self.lookup = {0:'N',1:'B',2:'G',3:'R'}
        self.storageArray = []
        self.stringAnalyzed = []
        self.max_value = 0
        self.light_pattern = "NNN"
        self.countThreshold = 2
        self.confidence_threshold = 0.5
        
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
        heartbeat = ["$RXCOD", self.light_pattern]
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
            "waypoint": tuple, # The target waypoint to set the GNC to (sending any other commands will override this)
            "end_mission": bool # Whether to end the mission
        }

        '''
        
        perc_cmd = {}
        gnc_cmd = {}
        # gnc_cmd = {"poshold": True}

        # Run one yolo inference on the provided frame at camera_data.frame
        center_camera_data = camera_data.get("center")
        center_camera_results = center_camera_data.results
        
        port_camera_data = camera_data.get("port")
        port_camera_results = port_camera_data.results

        starboard_camera_data = camera_data.get("starboard")
        starboard_camera_results = starboard_camera_data.results

        if center_camera_results is None or port_camera_results is None or starboard_camera_results is None:
            return perc_cmd, gnc_cmd
        
        res1 = self.image_process(center_camera_data, center_camera_results, "center") #None if error, false if end mission, other wise 0 to 3 number
        res2 = self.image_process(port_camera_data, port_camera_results, "port")
        res3 = self.image_process(starboard_camera_data, starboard_camera_results, "starboard")
        
        results = []
        results.extend([res1, res2, res3])
        
        for i in results:
            if i == None:
                return perc_cmd, gnc_cmd
            if i == False:
                # End mission
                gnc_cmd = {"end_mission": True}
                return perc_cmd, gnc_cmd
            if i in ["0","1","2","3"]:
                self.storageArray.append(i)
            
        if(len(self.storageArray) != 0):
            self.log(f"Interim processing.  Current storage array: {self.storageArray}")
            returnVal = self.filter(self.storageArray)
            records = self.analyze(returnVal)

            self.colors, self.pattern = self.results(records)
            # self.log(f"Results: {self.colors[0]}, {self.colors[1]}, {self.colors[2]}")

            
            max_vote = records.get(self.pattern)
            if max_vote is None:
                max_vote = 0
            if(max_vote > 0):
                if(max_vote >= self.countThreshold):
                    gnc_cmd = {"end_mission": True}
                else:
                    self.light_pattern = ''.join(self.colors)
            
        return perc_cmd, gnc_cmd

    def end(self):
        '''
        Any cleanup code can be placed here.
        This function is called when the run function returns end_mission = True 
        or when the mission handler decides to end the mission.
        '''
        cv2.destroyAllWindows()
        pass
        
    
    def analyze(self, stringAnalyzed):
        """Analyzes input string and adds votes to record

        Args:
            stringAnalyzed (String): raw string of grouped detections

        Returns:
            dictionary: Record of all votes for each possible pattern
        """
        records = {"123":0, "132":0, "213":0, "231":0, "312":0, "321":0 }

        

        val = "".join(stringAnalyzed)
        
        # self.log(f"Analysis started\nJoined: {val}")
        
        # Everything is now combined and we want to split on black (0)

        splitVal = val.split('0')

        # Updating all the counts in records
        for i in splitVal:
            if i in records:
                records[i] += 1
        
            
        # self.log(f"Analysis ended.\nRecords: {records}")

        return records
        
    
    def filter(self, storageArray, debug=False):
        """Filters string of inputs by lambda group function

        Args:
            storageArray (String Array): Raw array of detected objects

        Returns:
            String: Raw compressed string of filtered detections
        """
        # self.log(f"Filtering started")
        stringAnalyzed = []
            
        analyzed = [key for key, _group in groupby(storageArray)]
        for i in analyzed:
            stringAnalyzed.append(str(i))
                            
        # self.log(f"Filtering complete. Analysis: {analyzed}")
        return stringAnalyzed
    
    def results(self, records):
        """ Computes results of current vote at time of function call

        Args:
            records (dictionary): dictionary of votes

        Returns:
            ([string, string, string], int): list of three colors and number of votes it received
        """
        
        # self.log(f"Logging dictionary at start of results. {records}")

        pattern_count = max(records, key=records.get)
        max_counter = records.get(pattern_count)
        
        if (max_counter):
            voteCount = records.get(pattern_count)
            color1 = self.lookup[int(pattern_count[0])]
            color2 = self.lookup[int(pattern_count[1])]
            color3 = self.lookup[int(pattern_count[2])]
            self.log(f"Determined pattern based on the voting scheme from records: {pattern_count}, With vote of: {voteCount}\n, \
                     Individual colors: {color1}, {color2}, {color3}")

        
            return [color1, color2, color3], pattern_count
        
        else:
            self.log(f"No pattern cound in results")
            return ["N", "N", "N"], 0
        
    def classify_color(self, avg_color):
        b, g, r = avg_color  # OpenCV uses BGR format

        if np.all(avg_color < 100):  # Check if the color is approximately black
            return "0"
        elif r > g and r > b and r > 100:
            return "3"
        elif g > r and g > b and g > 100:
            return "2"
        elif b > r and b > g and b > 100:
            return "1"
        else:
            return f"Unknown Color: ({r}, {g}, {b})"
        
        
        # 0 is black
        # 1 is blue
        # 2 is green
        # 3 is red
        
        
        
    def image_process(self, cdata, cresults, name):
        
        # Run one yolo inference on the provided frame at camera_data.frame
        center_camera_data = cdata
        center_camera_results = cresults
        
        # Get the name of the highest confidence detection
        conf_list = []
        for result in center_camera_results:
            for box in result.boxes:
                conf = box.conf.item()
                cls_id = box.cls.item()
                x1, y1, x2, y2 = box.xyxy.cpu().numpy().flatten()
                
                # Append the class id, confidence, and bounding box coordinates to conf_list
                conf_list.append((int(cls_id), conf, (x1, y1, x2, y2)))
                        
        if len(conf_list) == 0:
            return None

        # Find the result with the highest confidence
        highest_confidence_result = max(conf_list, key=lambda x: x[1])
        
        # Extract the class ID, confidence, and bounding box coordinates
        bounding_box_coords = highest_confidence_result[2]
        
        x1, y1, x2, y2 = bounding_box_coords
        
        '''
        
        Assume model only gives you detection boxes with confidence > 0.5
        The coordinates of the box can be accessed like so:
            x1, y1, x2, y2 = box.xyxy.cpu().numpy().flatten()
        
        Take the frame crop it using the coordinates.
        Then blur the cropped image. Then crop to the inner 10% of the image (centered to the bounding box).
        If all the pixels are below a certain threshold, then the box is a black box.
        Otherwise, the box is a colored box.
        Take the average of the all the pixels above the threshold to determine the color (so you dont get black pixels from stripping).
        You can identify the color by seeing the largest value in the RGB tuple.
        '''
        
        # Crop to bounding box
        cropped_image = center_camera_data.frame[int(y1):int(y2), int(x1):int(x2)]
        
        # Get the center 20% of the cropped image
        height, width, _ = cropped_image.shape
        center_x1 = int(width * 0.40)
        center_y1 = int(height * 0.40)
        center_x2 = int(width * 0.60)
        center_y2 = int(height * 0.60)

        cropped_center = cropped_image[center_y1:center_y2, center_x1:center_x2]

        # Apply Gaussian blur
        blurred_image = cv2.GaussianBlur(cropped_center, (25, 25), 0)
        blurred_image = cv2.GaussianBlur(blurred_image, (25, 25), 0)

        test_f = cv2.resize(blurred_image, (200,200), interpolation=cv2.INTER_NEAREST)
        cv2.imshow(name, test_f)
        if cv2.waitKey(4) & 0xFF == ord('q'):
            gnc_cmd = {"end_mission": True}
            return False

        # Calculate the average color of the blurred image
        avg_color_per_row = np.mean(blurred_image, axis=0)
        avg_color = np.mean(avg_color_per_row, axis=0, dtype=np.int16)
        
        color_class = self.classify_color(avg_color)

        if color_class in ["0","1","2","3"]:
            self.log(f"Average color: {avg_color}, Classified color: {color_class}")
            return color_class
        else:
            self.log(color_class)
            return None
