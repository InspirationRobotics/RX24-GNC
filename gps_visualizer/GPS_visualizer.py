import smopy
import cv2
import numpy as np
from PIL import Image
from parse_data import DataParser
from datetime import datetime

class GPSVisualizer:

    def __init__(self, file_path, zoom, playback_speed, *, frame_size=600):
        dp = DataParser(file_path)
        self.position, self.heading = dp.parse_data()
        self.playback_speed = playback_speed
        self.zoom = zoom
        self.frame_size = frame_size
        self.get_map()
        
    def get_map(self, save=False):
        lat, lon = self.position[next(iter(self.position))]
        zoom = self.zoom
        map = smopy.Map((lat, lon, lat, lon), z=zoom)
        if save:
            map.save_png("map.png")
        self.map_obj = map
        frame = cv2.cvtColor(map.to_numpy(), cv2.COLOR_RGB2BGR)
        frame = cv2.resize(frame, (self.frame_size, self.frame_size), interpolation=cv2.INTER_LINEAR)
        # sharpen the frame
        frame = cv2.GaussianBlur(frame, (0, 0), 1.0)
        frame = cv2.addWeighted(frame, 1.5, frame, -0.5, 0)
        kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        frame = cv2.filter2D(frame, -1, kernel)
        self.frame = frame
        return map
    
    def convert_heading_to_arrow(self, heading, size = 10):
        # Assume top of image is north
        # Heading is in degrees, we need to create an arrow offset of x and y ratio
        # We will assume the arrow is 10 pixels long
        x = size * np.sin(np.radians(heading))
        y = size * np.cos(np.radians(heading))
        return int(x), int(-y)
    
    def rescale(self, x, y):
        size_w = self.map_obj.w
        size_h = self.map_obj.h
        x = int(x * self.frame_size / size_w)
        y = int(y * self.frame_size / size_h)
        return x, y

    def draw_test(self):
        x, y = self.map_obj.to_pixels(self.position[next(iter(self.position))])
        # scale the x,y to the frame size
        x, y = self.rescale(x, y)
        frame = self.frame.copy()
        frame = cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("map", frame)
        cv2.waitKey(0)

    def draw(self):
        for ts in self.position:
            x, y = self.map_obj.to_pixels(self.position[ts])
            x, y = self.rescale(x, y)
            # create a copy of the frame
            frame = self.frame.copy()
            # draw the red point underneath the blue point and on the saved frame
            self.frame = cv2.circle(self.frame, (x, y), 1, (0, 0, 255), -1)
            # draw the blue point on top of the frame
            frame = cv2.circle(frame, (x, y), 2, (255, 0, 0), -1)
            # draw the green heading arrow
            heading = self.heading.get(ts, None)
            if heading is not None:
                dx, dy = self.convert_heading_to_arrow(heading)
                x1, y1 = x + dx, y + dy
                frame = cv2.arrowedLine(frame, (x, y), (x1, y1), (0, 0, 0), 2, tipLength=0.5)
            # print the time on the frame
            stamp = datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            cv2.putText(frame, stamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.imshow("map", frame)
            if cv2.waitKey(int(1000/self.playback_speed)) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # file_path = "logs_6_06_24/GPSlog_1717697944.txt"
    file_path = "logs_6_06_24/GPSlog_1717701866.txt"
    playback_speed = 1000
    visualizer = GPSVisualizer(file_path, 19, playback_speed, frame_size=900)
    visualizer.draw()
