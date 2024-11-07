import os
os.environ['YOLO_VERBOSE'] = 'False'

import cv2
import time
import numpy as np
from mission_core import MissionHandler, STCMission, EntryMission

if __name__ == "__main__":

    mission1 = EntryMission(debug_mode=False)
    mission2 = STCMission()
    mission_handler = MissionHandler([mission1, mission2])
    mission_handler.start_mission()
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    
    cv2.destroyAllWindows()
    mission_handler.stop()
    
