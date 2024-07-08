from GPS_visualizer import GPSVisualizer

if __name__ == "__main__":
    # file_path = "logs_6_06_24/GPSlog_1717697944.txt"
    file_path = "logs_6_06_24/GPSlog_1717701866.txt"
    playback_speed = 1000
    zoom = 19
    visualizer = GPSVisualizer(file_path, zoom, playback_speed)
    visualizer.draw()