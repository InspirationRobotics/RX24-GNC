import time

class Heartbeat:
    def __init__(self, msg_id : str, team_id : str = "INSPX"):
        self.msg_id = msg_id
        # Time Stuff
        timestamp = time.time()
        self.EST_date = time.strftime("%d%m%y", time.localtime(timestamp)) # ddmmyy
        self.EST_time = time.strftime("%H%M%S", time.localtime(timestamp)) # HHMMSS (24-hour format)
        # Team Id
        self.team_id = team_id
        # Status
        self.status = []
        # Validity
        self.is_valid = True

    def __str__(self):
        return self._generate_msg()

    def _generate_msg(self):
        '''
        The order a heartbeat is made is the following:
        0. A dollar sign
        1. msg_id
        2. EST_date
        3. EST_time
        4. Remaining self variables (besides team_id)
        5. team_id
        6. All variables in the self.status list
        7. An asterisk
        8. Checksum (XOR of all characters in the message) as a two-digit hex number
        9. <cr><lf>
        '''
        msg = f"{self.msg_id},{self.EST_date},{self.EST_time},"
        for name, val in self.__dict__.items():
            if name not in ["msg_id", "EST_date", "EST_time", "team_id", "status", "is_valid"]:
                msg += f"{val},"
        msg += f"{self.team_id},"
        for val in self.status:
            msg += f"{val},"
        # Remove the last comma
        msg = msg[:-1]
        checksum = 0
        for char in msg:
            checksum ^= ord(char)
        checksum = hex(checksum)[2:]
        msg += f"*{checksum}\r\n"
        return f"${msg}"


class SystemHeartbeat(Heartbeat):
    def __init__(self, position : tuple, mode : str, team_id : str = "INSPX"):
        if position is None:
            self.is_valid = False
            return
        super().__init__("RXHRB", team_id)
        # Location Stuff
        self.lat = str(abs(position[0]))
        self.lat_dir = "N" if position[0] >= 0 else "S"
        self.lon = str(abs(position[1]))
        self.lon_dir = "E" if position[1] >= 0 else "W"
        # Mode (1 is tele, 2 is auto, 3 is killed)
        mode = mode
        # UAV status
        uav_status = "1" # we are not using UAV
        self.status = [mode, uav_status]

class MissionHeartbeat(Heartbeat):
    def __init__(self, mission_state : list, team_id : str = "INSPX"):
        if len(mission_state) < 1:
            self.is_valid = False
            return
        super().__init__(mission_state[0], team_id)
        self.status = mission_state[1:]
        self.is_valid = True


if __name__ == "__main__":
    hb = SystemHeartbeat((42.0, -71.0), "1")
    print(hb)
    hb2 = MissionHeartbeat(["RXCOD", "RBG"])
    print(hb2)