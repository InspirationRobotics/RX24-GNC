import cv2
import json
import numpy as np
from typing import List

class Thruster:
    def __init__(self, name, position, angle, max_thrust):
        self.name = name
        self.position = position
        self.angle = angle
        self.max_thrust = max_thrust
        self.thrust = 0.0

    def set_thrust(self, thrust):
        self.thrust = np.clip(thrust, -self.max_thrust, self.max_thrust)

    def set_normalized_thrust(self, normalized_thrust):
        self.set_thrust(normalized_thrust * self.max_thrust)

    def get_dynamics(self, vehicle_orientation):
        force = self.thrust * np.array([np.cos(self.angle + vehicle_orientation), np.sin(self.angle + vehicle_orientation)])
        torque = np.cross(self.position, force)
        return force, torque
    
class Boat:
    def __init__(self, config_file):
        self.run_config(config_file)

    def run_config(self, config_file):
        with open(config_file, 'r') as f:
            config = json.load(f)
        
        self.name = config['name']
        vehicle_config = config['vehicle']

        self.position = np.array(vehicle_config['initial_position'])
        self.orientation = vehicle_config['initial_orientation']
        self.dt = vehicle_config['time_step']
        self.thruster_count = vehicle_config['thruster_count']
        
        thruster_configs = vehicle_config['thruster_config']
        self.thrusters : List[Thruster]= []
        for id in range(self.thruster_count):
            thruster_config = thruster_configs[str(id)]
            thruster = Thruster(
                thruster_config['name'],
                np.array(thruster_config['position']),
                thruster_config['angle'],
                thruster_config['thrust']
            )
            self.thrusters.append(thruster)


    def update(self):
        net_force = np.array([0.0, 0.0])
        net_torque = 0.0
        
        for thruster in self.thrusters:
            force, torque = thruster.get_dynamics(self.orientation)
            net_force += force
            net_torque += torque
        
        self.position += net_force * self.dt
        self.orientation += net_torque * self.dt

if __name__ == "__main__":
    boat = Boat("config.json")
