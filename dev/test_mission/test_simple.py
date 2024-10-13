import cv2
import time
import numpy as np
from mission_core import MissionHandler, SimpleMission, OccupancyData


if __name__ == "__main__":

    def visualize_occupancy(occupancy: OccupancyData):
        if occupancy is None:
            return
        x_size = occupancy.x_range[1] - occupancy.x_range[0] + 1 + 60
        y_size = occupancy.y_range[1] - occupancy.y_range[0] + 1 + 60
        frame = np.zeros((y_size, x_size))

        def translate(coord):
            x = coord[0] - occupancy.x_range[0] + 30
            y = y_size - (coord[1] - occupancy.y_range[0] + 30)
            return x, y

        for coord in occupancy.cells:
            if occupancy.cells[coord] <= occupancy.val_zero_point:
                continue
            x, y = translate(coord)
            frame[y, x] = occupancy.cells[coord] / occupancy.val_range[1]

        frame = np.array(frame * 255, dtype=np.uint8)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        frame = cv2.circle(frame, translate((0, 0)), 3, (0, 255, 0), -1)
        show_frame = cv2.resize(frame, (800,800), interpolation=cv2.INTER_NEAREST)
        cv2.imshow("Occupancy Grid", show_frame)
        cv2.waitKey(1) & 0xFF

    mission = SimpleMission()
    mission_handler = MissionHandler(mission)
    mission_handler.start_mission()
    while True:
        try:
            occupancy = mission_handler._get_occupancy()
            visualize_occupancy(occupancy)
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    
    cv2.destroyAllWindows()
    mission_handler.stop()
    
