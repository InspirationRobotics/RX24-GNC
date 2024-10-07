class SimpleMission:
    def __init__(self):
        self.end = False
        self.state = 0
        self.target_position = (32.7157, -117.1611)  # Example target position (lat, lon)

    def run(self, frame, position, og):
        """
        Executes the mission logic.
        :param frame: Latest camera frame.
        :param position: Current position of the boat.
        :param og: Occupancy grid data.
        :return: Dictionary of commands.
        """
        print("Running Simple Mission")
        if self.state == 0:
            # Move toward the target
            print(f"Moving toward {self.target_position}")
            output = {
                "target_position": self.target_position,
                "mission_state": "Moving to target"
            }
            # Check if we reached the target (simplified condition)
            if self.reached_target(position):
                self.state = 1  # Switch to next state
            return output
        
        elif self.state == 1:
            # End the mission
            self.end = True
            return {"mission_state": "Mission Complete"}

    def reached_target(self, position):
        """
        Check if the boat has reached the target position (simple distance check).
        """
        return position == self.target_position