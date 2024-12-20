import os
os.environ['YOLO_VERBOSE'] = 'False'

import cv2
import time
from mission_core import MissionHandler, FTPMission

if __name__ == "__main__":

    mission = FTPMission(left_color="R", debug_mode=True)
    mission_handler = MissionHandler(mission)
    mission_handler.start_mission()
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break

    cv2.destroyAllWindows()
    mission_handler.stop()
    
