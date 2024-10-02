# Just a simple test to check if the motor_core module is working properly
# We test by sending a command to the client to move the motor

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


import time

from typing import Dict, Any, List, Tuple
from comms_core import Server, Logger
from comms_core import CustomSocketMessage as csm

MANUAL = 0
POSHOLD = 1
WAYPOINT = 2

def set_mode(mode, msg : Dict[str, Any]):
   msg['set_mode'] = mode
   return msg

def set_target_vector(vector : Tuple[float, float], msg : Dict[str, Any]):
   msg['target_vector'] = vector
   return msg

def set_target_position(position : Tuple[float, float], msg : Dict[str, Any]):
   msg['target_position'] = position
   return msg

def set_target_heading(heading : float, msg : Dict[str, Any]):
   msg['target_heading'] = heading
   return msg

def emergency_stop(stop : bool, msg : Dict[str, Any]):
   msg['emergency_stop'] = stop
   return msg

def set_debug(debug : bool, msg : Dict[str, Any]):
   msg['debug'] = debug
   return msg

def set_raw_motor_power(power : Tuple[float, float], msg : Dict[str, Any]):
   msg['raw_motor_power'] = power
   return msg
    
def dummy_callback(data, addr):
   pass
   # print(f"Got data from {addr}")

# Create a server object
server = Server(default_callback=dummy_callback)
server.start()

# Create a logger object
logger = Logger("test_control")



def process_input(data, msg):
   if data == "exit" or data == "quit":
      return False

# Create an interactive terminal to send commands to the server

# Available commands:
# - 1. set_mode -> 0, 1, 2
# - 2. set_target_vector -> [x, y]
# - 3. set_target_position -> [lat, lon]
# - 4. set_target_heading -> heading
# - 5. emergency_stop -> True, False
# - 6. set_debug -> True, False
# - 7. set_raw_motor_power -> [left_pwr, right_pwr] (-1 to 1)

def ask_for_input(msg):
   data = input(msg)
   if data == "exit" or data == "quit":
      return False
   if data == "back":
      return None
   return data

def process_input(data, to_send : dict):

   if data == "1":
      mode = ask_for_input("Enter the mode (0, 1, 2): ")
      if mode == False or mode == None:
         return mode
      to_send = set_mode(int(mode), to_send)
      return to_send
   
   if data == "2":
      x = ask_for_input("Enter the target x-vector: ")
      if x == False or x == None:
         return x
      y = ask_for_input("Enter the target y-vector: ")
      if y == False or y == None:
         return y
      to_send = set_target_vector([float(x), float(y)], to_send)
      return to_send
   
   if data == "3":
      # lat = ask_for_input("Enter the target latitude: ")
      # if lat == False or lat == None:
      #    return lat
      # lon = ask_for_input("Enter the target longitude: ")
      # if lon == False or lon == None:
      #    return lon
      #32.91426471087947, -117.10186806192723

      # lat = 32.91426471087947
      # lon = -117.10186806192723

      point1 = (32.91514313674598, -117.10053347345651)
      point2 = (32.91519019795828, -117.10160586313923)
      point3 = (32.91474180227439, -117.10084539786419)
      point4 = (32.91469620258607, -117.10029768180299)  
      points = [point1, point2, point3, point4]
      cnt = -1
      while True:
         val = input()
         if val == " ":
            cnt+=1
         if val == "q":
            break
         if cnt > 3:
            break
         lat, lon = points[cnt]
         to_send = set_target_position([float(lat), float(lon)], to_send)
         server.send(csm.encode(to_send))
      return {}
   
   if data == "4":
      heading = ask_for_input("Enter the target heading: ")
      if heading == False or heading == None:
         return heading
      to_send = set_target_heading(float(heading), to_send)
      return to_send
   
   if data == "5":
      stop = ask_for_input("Enter True or False: (1, 0)")
      if stop == False or stop == None:
         return stop
      stop = True if stop == "1" else False
      to_send = emergency_stop(stop, to_send)
      return to_send
   
   if data == "6":
      debug = ask_for_input("Enter True or False: (1, 0)")
      if debug == False or debug == None:
         return debug
      debug = True if debug == "1" else False
      to_send = set_debug(debug, to_send)
      return to_send
   
   if data == "7":
      left = ask_for_input("Enter the left motor power: (-1 to 1)")
      if left == False or left == None:
         return left
      right = ask_for_input("Enter the right motor power: (-1 to 1)")
      if right == False or right == None:
         return right
      to_send = set_raw_motor_power([float(left), float(right)], to_send)
      return to_send
   
   if data == "exit" or data == "quit":
      return False

   if data == "send":
      server.send(csm.encode(to_send))
      if "target_position" in to_send.keys() or "target_heading" in to_send.keys():
         while input() != "q":
            server.send(csm.encode(to_send))
      return {}
    
   return False

while True:
   print("Welcome to the motor control test terminal!")
   print("Available commands:")
   print("1. set_mode -> 0, 1, 2")
   print("2. set_target_vector -> [x, y]")
   print("3. set_target_position -> [lat, lon]")
   print("4. set_target_heading -> heading")
   print("5. emergency_stop -> True, False")
   print("6. set_debug -> True, False")
   print("7. set_raw_motor_power -> [left_pwr, right_pwr] (-1 to 1)")
   print("send -> Send the command to the server")
   print("exit or quit-> Exit the terminal")
   print("back -> Go back to this screen")
   print("Enter the command: ")
   to_send = {}
   while True:
      data = ask_for_input(">> ")
      if data == False:
         print("Exiting the terminal...")
         server.stop()
         exit()
      to_send = process_input(data, to_send)
      if to_send == False:
         print("Exiting the terminal...")
         server.stop()
         exit()
      if to_send == None:
         break


'''
32.9513
-117.10118
'''