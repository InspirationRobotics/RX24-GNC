import os
os.environ['YOLO_VERBOSE'] = 'False'

import cv2
import time
import numpy as np
from mission_core import MissionHandler, BasicEntry, FTPMission, DockMission

if __name__ == "__main__":

    mission0 = BasicEntry()
    mission1 = FTPMission(start_waypoint=(0,0), start_heading=0)
    mission2 = DockMission()
    mission_handler = MissionHandler([mission0, mission1, mission2])
    mission_handler.start_mission()
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    
    cv2.destroyAllWindows()
    mission_handler.stop()
    
