'''
Taken from motor_core.py. Details the available commands for the motor_core module.

Global Interface:
 - This data is received from the server and is purely commands. It expires in 2 seconds.
 - If the interface timestamp is expired by over 10 seconds, all motors will be killed.

 - debug: A boolean that is persistant. If in debug mode none of the outputs will react
    - raw_motor_power: You can directly send motor commands using this while in debug
 - target_vector: A list with 2 values, the x and y components of the target vector.
    - If not in waypoint or poshold mode, this is the target vector.
 - target_position: A list with 2 values, the lat and lon components of the target position.
    - If in waypoint mode, this is the target position. This is persistent until used.
 - mission_name: A string with the mission name. If a valid name is given, the waypoints will be executed sequentially
 - target_heading: A single value, the target heading of the rover.
    - If in poshold or manual mode, this is the target heading.
 - set_mode: A single value, the target mode of the rover.
    - This is persistent, meaning that it will not expire until a new value is received.
    - If 0, the rover should be in manual mode.
    - If 1, the rover should be in poshold mode.
    - If 2, the rover should be in waypoint mode.
 - emergency_stop: A single value, if True, the rover should stop all motors. Until a False is received, the rover will not move.
     - This is a persistent command, meaning that it will not expire until a zero is received.

'''

# This is similar to test_control.py, but this has an inbuilt heartbeat, so the gi never times out in motor_core
# It also has simpler functionality allowing for the following features:
# - Set mode
# - Set target vector
# - Set target position
# - Set target heading
# - Send mission name (for waypoint mode)

# The heartbeat is sent every 1.5 seconds and is an unused li variable: heartbeat
# When additional data needs to be sent, it is just added on to the heartbeat
# Only one command can be added to the heartbeat at a time

import time

from threading import Thread, Lock
from typing import Dict, Any, List, Tuple

from comms_core import Server, Logger
from comms_core import CustomSocketMessage as csm

MANUAL = 0
POSHOLD = 1
WAYPOINT = 2

class TestControl(Logger):
    
   def __init__(self):
      super().__init__(__name__)
      self.server = Server()
      self.server.start()

      self.msg_lock = Lock()
      self.msg = {}

      self.active = True
      self.heartbeat_thread = Thread(target=self.send_hearbeat_thread, daemon=True)

      self.heartbeat_thread.start()

   def stop(self):
      self.active = False
      self.heartbeat_thread.join(3)
      self.server.stop()

   def __del__(self):
      self.stop()

   def set_mode(self, mode : str):
      try:
         mode = int(mode)
      except ValueError:
         self.error(f"Invalid mode {mode}. Mode should be 0, 1, or 2")
         return
      if mode not in [0, 1, 2]:
         self.error(f"Invalid mode {mode}. Mode should be 0, 1, or 2")
      with self.msg_lock:
         self.msg = {}
         self.msg['set_mode'] = mode

   def set_target_vector(self, vector : Tuple[str, str]):
      try:
         x = float(vector[0])
         y = float(vector[1])
      except ValueError:
         self.error(f"Invalid vector {vector}.")
         return
      with self.msg_lock:
         self.msg = {}
         self.msg['target_vector'] = [x,y]

   def set_target_position(self, position : Tuple[str, str]):
      try:
         x = float(position[0])
         y = float(position[1])
      except ValueError:
         self.error(f"Invalid position {position}.")
      with self.msg_lock:
         self.msg = {}
         self.msg['target_position'] = [x, y]

   def set_target_heading(self, heading : str):
      try:
         heading = float(heading)
      except ValueError:
         self.error(f"Invalid heading {heading}.")
      with self.msg_lock:
         self.msg = {}
         self.msg['target_heading'] = heading

   def set_mission_name(self, mission_name : str):
      with self.msg_lock:
         self.msg = {}
         self.msg['mission_name'] = mission_name

   def set_emergency_stop(self, stop : str):
      try:
         stop = int(stop)
         if stop not in [0, 1]:
            raise ValueError
      except ValueError:
         self.error(f"Invalid emergency stop {stop}.")
      stop = bool(stop)
      with self.msg_lock:
         self.msg = {}
         self.msg['emergency_stop'] = stop

   def send_hearbeat_thread(self):
      while self.active:
         with self.msg_lock:
            self.msg['heartbeat'] = 00
            self.server.send(csm.encode(self.msg))
            self.msg = {}
         time.sleep(1.5)

   def process_input(self, data):

      if data == "exit" or data == "quit":
         return False
      
      if data == "1":
         mode = input("Enter the mode (0, 1, 2): ")
         self.set_mode(mode)
   
      if data == "2":
         x = input("Enter the target x-vector: ")
         y = input("Enter the target y-vector: ")
         self.set_target_vector([x, y])

      if data == "3":
         lat = input("Enter the target latitude: ")
         lon = input("Enter the target longitude: ")
         self.set_target_position([lat, lon])

      if data == "4":
         heading = input("Enter the target heading: ")
         self.set_target_heading(heading)

      if data == "5":
         mission_name = input("Enter the mission name: ")
         self.set_mission_name(mission_name)

      if data == "6":
         stop = input("Enter the emergency stop (0, 1): ")
         self.set_emergency_stop(stop)

      return True

   def handle_input(self):
      print("Welcome to the motor control test terminal!")
      print("Available commands:")
      print("1. set_mode -> 0, 1, 2")
      print("2. set_target_vector -> [x, y]")
      print("3. set_target_position -> [lat, lon]")
      print("4. set_target_heading -> heading")
      print("5. Mission Name -> mission_name")
      print("6. emergency_stop -> True, False")
      print("exit or quit-> Exit the terminal")
      print("Enter the command: ")

      while True:
         data = input(">>> ")
         if not self.process_input(data):
            break
      print("Exiting...")

if __name__ == "__main__":
   control = TestControl()
   control.handle_input()
   control.stop()
