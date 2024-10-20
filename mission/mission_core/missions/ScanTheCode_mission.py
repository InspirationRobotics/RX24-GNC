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


class SimpleMission:

    # Define the initial perception commands here (if any)
    # This dictionary must exist, but some commands are entered as examples
    init_perc_cmd = {
        "start": ["center"],
        "stop": ["port", "starboard"],
        "record": ["center"],
        "stop_record": ["port", "starboard"],
        "load_model": [("center", "ML_Resources/stcA.pt")],
    }

    def __init__(self):
        self.records = {"123":0, "132":0, "213":0, "231":0, "312":0, "321":0 }
        self.lookup = {0:'N',1:'B',2:'G',3:'R'}
        self.storageArray = []
        self.stringAnalyzed = []
        self.max_value = 0
        self.light_pattern = ["NNN"]
        self.countThreshold = 3
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
        
        # Run one yolo inference on the provided frame at camera_data.frame
        center_camera_data = camera_data.get("center")
        center_camera_results = center_camera_data.results
        confidences = {}
        
        if center_camera_results is None:
            center_camera_results = self.get_latest_model_results()
        for result in center_camera_results:
            for box in result.boxes:
                conf = box.conf.item()
                if conf < self.confidence_threshold:
                    continue
                cls_id = box.cls.item()
                if cls_id not in confidences or conf > confidences[cls_id]:
                    confidences[cls_id] = conf
                    
        highest_confidence_name = max(confidences, key=confidences.get)
        highest_confidence_value = confidences[highest_confidence_name]
                        
                
        self.storageArray.append(highest_confidence_name)
            
        if(len(self.storageArray) != 0):
            self.log(f"Interim processing.  Current storage array: {self.storageArray}")
            returnVal = filter(self.storageArray)
            records = self.analyze(returnVal)

            self.colors, self.pattern = self.results(records)
            self.log(f"Results: {self.colors[0]}, {self.colors[1]}, {self.colors[2]}")

            
            max_vote = records.get(self.pattern)
            if max_vote is None:
                max_vote = 0
            if(max_vote > 0):
                if(max_vote >= self.countThreshold):
                    gnc_cmd = {"end_mission": True}
                else:
                    self.light_pattern = ''.join(self.colors)
            
            else:
                # No Colors to update, can just say black
                self.light_pattern = "NNN"
            
        perc_cmd = {}
        gnc_cmd = {"poshold": True}
        return perc_cmd, gnc_cmd

    def end(self):
        '''
        Any cleanup code can be placed here.
        This function is called when the run function returns end_mission = True 
        or when the mission handler decides to end the mission.
        '''
        pass
        
    
    def analyze(self, stringAnalyzed, debug=False):
        """Analyzes input string and adds votes to record

        Args:
            stringAnalyzed (String): raw string of grouped detections

        Returns:
            dictionary: Record of all votes for each possible pattern
        """
        records = {"123":0, "132":0, "213":0, "231":0, "312":0, "321":0 }

        

        val = "".join(stringAnalyzed)
        
        self.log(f"Analysis started\nJoined: {val}")
        
        # Everything is now combined and we want to split on black (0)

        splitVal = val.split('0')

        # Updating all the counts in records
        for i in splitVal:
            if i in records:
                records[i] += 1
            else:
                if(debug):
                    self.log(f"Ignoring invalid key: {i}")
        
            
        self.log(f"Analysis ended.\nRecords: {records}")

        return records
        
    
    def filter(self, storageArray, debug=False):
        """Filters string of inputs by lambda group function

        Args:
            storageArray (String Array): Raw array of detected objects

        Returns:
            String: Raw compressed string of filtered detections
        """
        self.log(f"Filtering started")
        stringAnalyzed = []
            
        analyzed = [key for key, _group in groupby(storageArray)]
        for i in analyzed:
            stringAnalyzed.append(str(i))
                            
        self.log(f"Filtering complete. Analysis: {analyzed}")
        return stringAnalyzed
    
    def results(self, records):
        """ Computes results of current vote at time of function call

        Args:
            records (dictionary): dictionary of votes

        Returns:
            ([string, string, string], int): list of three colors and number of votes it received
        """
        
        self.log(f"Logging dictionary at start of results. {records}")

        pattern_count = max(records, key=records.get)
        max_counter = records.get(pattern_count)
        
        if (max_counter):
            voteCount = records.get(pattern_count)
            color1 = self.lookup[int(pattern_count[0])]
            color2 = self.lookup[int(pattern_count[1])]
            color3 = self.lookup[int(pattern_count[2])]
            self.log(f"Determined pattern based on the voting scheme from records: {pattern_count}, With vote of: {voteCount}\n,
                     Individual colors: {color1}, {color2}, {color3}")

        
            return [color1, color2, color3], pattern_count
        
        else:
            self.log(f"No pattern cound in results")
            return ["N", "N", "N"], 0
