import os
os.environ['YOLO_VERBOSE'] = 'False'

import cv2
import time
import numpy as np
from mission_core import MissionHandler, STCMission

if __name__ == "__main__":

    mission = STCMission()
    mission_handler = MissionHandler(mission)
    mission_handler.start_mission()
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    
    cv2.destroyAllWindows()
    mission_handler.stop()
    
