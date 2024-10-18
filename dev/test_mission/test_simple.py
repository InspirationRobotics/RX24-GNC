import cv2
import time
import numpy as np
from mission_core import MissionHandler, SimpleMission

if __name__ == "__main__":

    mission = SimpleMission()
    mission_handler = MissionHandler(mission)
    mission_handler.start_mission()
    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    
    cv2.destroyAllWindows()
    mission_handler.stop()
    
