from pathlib import Path
import time

class DataParser:

    def __init__(self, file_path):
        self.file_path = file_path
        self.position = {}
        self.heading = {}

    def parse_gps_data(self, data : str):
        data = data[1:-1]
        data = data.split(', ')
        data = [float(i) for i in data]
        return tuple(data)
        
    def parse_heading_data(self, data : str):
        return float(data)

    def parse_data(self):
        with open(self.file_path, 'r') as file:
            for line in file:
                line = line.strip()
                time_stamp, data = line.split('__: ', 1)
                time_stamp = int(time_stamp)
                if "(" in data:
                    self.position[time_stamp] = self.parse_gps_data(data)
                else:
                    self.heading[time_stamp] = self.parse_heading_data(data)
        return self.position, self.heading
                

if __name__ == "__main__":
    file_path = Path("logs_6_06_24/GPSlog_1717697944.txt")
    data_parser = DataParser(file_path)
    print(data_parser.position)
    #print(data_parser.heading)