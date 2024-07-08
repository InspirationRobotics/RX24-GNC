from GPS_visualizer import GPSVisualizer

if __name__ == "__main__":
    file_path = "logs_6_06_24/GPSlog_1717697944.txt"
    # file_path = "logs_6_06_24/GPSlog_1717701866.txt"
    visualizer = GPSVisualizer(file_path)
    visualizer.draw()