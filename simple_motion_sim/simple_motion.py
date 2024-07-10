import cv2
import json
import numpy as np
from typing import List
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

class Thruster:
    def __init__(self, name, position, angle, max_thrust):
        self.name = name
        self.position = position
        self.rad_angle = np.radians(angle)
        self.max_thrust = max_thrust
        self.thrust = 0.0

    def set_thrust(self, thrust):
        self.thrust = np.clip(thrust, -self.max_thrust, self.max_thrust)

    def set_normalized_thrust(self, normalized_thrust):
        self.set_thrust(normalized_thrust * self.max_thrust)

    def get_dynamics(self):
        force = self.thrust * np.array([np.sin(self.rad_angle), np.cos(self.rad_angle)])
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

        self.max_thrust = max([t.max_thrust for t in self.thrusters])

    def full_throttle(self):
        for thruster in self.thrusters:
            thruster.set_thrust(1.0)

    def strafe_right(self):
        self.thrusters[0].set_thrust(-1.0) #fr
        self.thrusters[1].set_thrust(1.0) #fl
        self.thrusters[2].set_thrust(1.0) #br
        self.thrusters[3].set_thrust(-1.0) #bl

    def rotate_cc(self):
        self.thrusters[0].set_thrust(1.0) #fr
        # self.thrusters[1].set_normalized_thrust(-1.0) #fl
        # self.thrusters[2].set_normalized_thrust(1.0) #br
        self.thrusters[3].set_thrust(-1.0) #bl

    def arc_right(self):
        self.thrusters[0].set_thrust(1.0)
        # self.thrusters[1].set_thrust(1.0)
        # self.thrusters[2].set_thrust(1.0)
        # self.thrusters[3].set_thrust(0.0)

    def update(self):
        net_force = np.array([0.0, 0.0])
        net_torque = 0.0
        
        for thruster in self.thrusters:
            force, torque = thruster.get_dynamics()
            net_force += force
            net_torque += torque 
        
        self.position += net_force * self.dt
        self.orientation += net_torque * self.dt
        print(f'Position: {self.position}, Orientation: {self.orientation}')
        print(f'Net Force: {net_force}, Net Torque: {net_torque}')


class simple_motion_sim:
    def __init__(self, config_file):
        self.boat = Boat(config_file)
        self.setup_plot()
        self.boat.rotate_cc()
        self.run_simulation()

    def setup_plot(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        self.boat_patch = Rectangle(self.boat.position - 0.5, 1, 1, angle=0.0, fc='blue', rotation_point='center')
        self.ax.add_patch(self.boat_patch)

    def update(self, frame):
        self.boat.update()
        self.boat_patch.set_xy(self.boat.position - 0.5)
        self.boat_patch.angle = np.degrees(self.boat.orientation)
        return self.boat_patch,

    def run_simulation(self):
        ani = FuncAnimation(self.fig, self.update, frames=range(100), blit=True, interval=100)
        plt.show()


if __name__ == "__main__":
    sim = simple_motion_sim("configs/simple_config.json")
