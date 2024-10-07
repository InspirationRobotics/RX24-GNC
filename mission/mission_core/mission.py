import time
from threading import Thread, Lock
from typing import Dict, List, Any

class MissionHandler:
    def __init__(self, mission, perception, gnc_control):
        """
        Initialize the MissionHandler.
        :param mission: The current mission object (e.g., GoToTarget, SlalomCourse, etc.).
        :param perception: The perception class (e.g., cameras, lidar).
        :param gnc_control: The GNC (unsure how this works)
        """
        self.mission = mission  # The current mission
        self.perception = perception  # Perception system (cameras, CV models)
        self.gnc_control = gnc_control  # Control system (motors, navigation)
        self.occupancy_grid = None  # Placeholder for occupancy grid
        self.position = None  # Current position of the boat
        self.frame = None  # Current camera frame
        
        # Heartbeat and locking
        self.heartbeat_active = True
        self.msg_lock = Lock()
        self.msg = {}
        
        # Heartbeat thread to periodically send status messages
        self.heartbeat_thread = Thread(target=self._send_heartbeat, daemon=True)
        self.heartbeat_thread.start()

    def load_occupancy_grid(self):
        """
        Placeholder for loading the occupancy grid.
        """
        # TODO: Load occupancy grid here (unsure how to do this)
        self.occupancy_grid = "Simulated occupancy grid data"

    def position_callback(self, position):
        """
        Update the current boat position.
        :param position: Tuple containing the latitude and longitude.
        """
        with self.msg_lock:
            self.position = position

    def frame_callback(self, frame):
        """
        Update the latest frame from the perception system.
        :param frame: The current camera frame.
        """
        with self.msg_lock:
            self.frame = frame

    def run(self):
        """
        Main loop that runs the current mission.
        """
        while not self.mission.end:
            # Update perception and occupancy grid data
            self.frame_callback(self.perception.get_latest_frames())
            self.load_occupancy_grid()  # To dynamically update, could be implemented incorrectly
            
            # Execute the mission logic
            with self.msg_lock:
                output = self.mission.run(self.frame, self.position, self.occupancy_grid)
            
            # Process the output and send relevant commands to GNC system
            self.send_commands(output)

            # For now Users will specify when to switch to a new mission
            time.sleep(1)  # Add some delay to avoid running too fast

    def send_commands(self, output: Dict[str, Any]):
        """
        Send the relevant commands from the mission output to the GNC system.
        :param output: Dictionary containing the commands (target vector, heading, etc.).
        TODO: need to talk about what exactly needs to be sent between
        """
        with self.msg_lock:
            if "target_vector" in output:
                self.gnc_control.set_target_vector(output["target_vector"])
            if "target_position" in output:
                self.gnc_control.set_target_position(output["target_position"])
            if "target_heading" in output:
                self.gnc_control.set_target_heading(output["target_heading"])
            if "mission_state" in output:
                print(f"Mission state: {output['mission_state']}")

    def _send_heartbeat(self):
        """
        Heartbeat thread that periodically sends system status.
        TODO: what should be in a heartbeat message? Should there be a heart beat between GNC and Misison? 
        """
        while self.heartbeat_active:
            with self.msg_lock:
                self.msg['heartbeat'] = "alive"
                self.msg['current_position'] = self.position
                self.msg['current_mission'] = type(self.mission).__name__
                self.gnc_control.send_heartbeat(self.msg)
            
            time.sleep(1.5)  # Heartbeat interval

    def stop_heartbeat(self):
        """
        Stops the heartbeat thread when the MissionHandler is no longer needed.
        """
        self.heartbeat_active = False
        self.heartbeat_thread.join()
