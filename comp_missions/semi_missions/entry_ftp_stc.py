import os
os.environ['YOLO_VERBOSE'] = 'False'

import cv2
import time
import numpy as np
from mission_core import MissionHandler, BasicEntry, FTPMission, PreSTCMission, STCMission, DockMission, BasicExit

if __name__ == "__main__":

    mission0 = BasicEntry(distance=42, start_waypoint=(27.375189, -82.452475))
    mission1 = FTPMission(left_color="R", start_waypoint=(27.37535299868433, -82.45309264065489), start_heading=280)
    mission2 = PreSTCMission(start_waypoint=(27.37523396312006, -82.45338645963143), start_heading=115 )
    mission3 = STCMission()
    mission4 = DockMission()
    mission5 = BasicExit(waypoints=[(0,0), (0,0)])
    mission_handler = MissionHandler([mission0, mission1, mission2, mission3, mission4, mission5])
    mission_handler.start_mission()
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    
    cv2.destroyAllWindows()
    mission_handler.stop()
    
