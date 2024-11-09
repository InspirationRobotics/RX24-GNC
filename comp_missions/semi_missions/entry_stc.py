import os
os.environ['YOLO_VERBOSE'] = 'False'

import cv2
import time
import numpy as np
from mission_core import MissionHandler, STCMission, PreSTCMission, BasicEntry, DockMission, BasicExit

if __name__ == "__main__":

    mission0 = BasicEntry(distance=43, start_waypoint=(27.373318, -82.452442))
    mission1 = PreSTCMission(debug_mode=False)
    mission2 = STCMission()
    mission3 = DockMission()
    mission4 = BasicExit(waypoints=[(0,0), (0,0)])
    mission_handler = MissionHandler([mission0, mission1, mission2, mission3, mission4])
    mission_handler.start_mission()
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    
    cv2.destroyAllWindows()
    mission_handler.stop()
    
