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

    def get_dynamics(self, thrust : float = None):
        if thrust is not None:
            thrust = np.clip(thrust, -self.max_thrust, self.max_thrust)
        else:
            thrust = self.thrust
        force = thrust * 4.44822 # convert to newtons from lbf
        force *= np.array([np.sin(self.rad_angle), np.cos(self.rad_angle)])
        torque = np.cross(self.position, force)
        return force, torque
    
class LinkedThrusters:

    def __init__(self, name : str, thruster1 : Thruster, thruster2 : Thruster):
        self.name = name
        self.thruster1 = thruster1
        self.thruster2 = thruster2
        self.max_thrust = thruster1.max_thrust + thruster2.max_thrust
        self.thrust1_ratio = thruster1.max_thrust / self.max_thrust
        self.thrust2_ratio = thruster2.max_thrust / self.max_thrust
        self.thrust = 0.0

    def set_thrust(self, thrust):
        self.thrust = np.clip(thrust, -self.max_thrust, self.max_thrust)
        self.thruster1.set_thrust(thrust * self.thrust1_ratio)
        self.thruster2.set_thrust(thrust * self.thrust2_ratio)

    def set_normalized_thrust(self, normalized_thrust):
        self.set_thrust(normalized_thrust * self.max_thrust)

    def get_dynamics(self, thrust : float = None):
        if thrust is not None:
            thrust = np.clip(thrust, -self.max_thrust, self.max_thrust)
        else:
            thrust = self.thrust
        force1, torque1 = self.thruster1.get_dynamics(thrust * self.thrust1_ratio)
        force2, torque2 = self.thruster2.get_dynamics(thrust * self.thrust2_ratio)
        return force1 + force2, torque1 + torque2
    
class Boat:
    def __init__(self, config_file):
        self.run_config(config_file)

    def run_config(self, config_file):
        with open(config_file, 'r') as f:
            config = json.load(f)
        
        self.name = config['name']
        vehicle_config : dict = config['vehicle']

        self.position = np.array(vehicle_config.get('initial_position', [0.0, 0.0]))
        self.orientation = vehicle_config.get('initial_orientation', 0.0)
        self.dt = vehicle_config.get('time_step', 0.1)
        self.thruster_count = vehicle_config.get('thruster_count', 4)

        links = vehicle_config.get("links", None)
        
        thruster_configs = vehicle_config['thruster_config']
        thrusters : List[Thruster]= []
        for id in range(self.thruster_count):
            thruster_config : dict = thruster_configs[str(id)]
            thruster = Thruster(
                thruster_config.get('name', f'thruster_{id}'),
                np.array(thruster_config['position']),
                thruster_config['angle'],
                thruster_config['thrust']
            )
            thrusters.append(thruster)

        to_pop = []
        if links is not None:
            for link in links.values():
                linked_thrusters = LinkedThrusters(
                    link['name'],
                    thrusters[link['indexes'][0]],
                    thrusters[link['indexes'][1]]
                )
                thrusters[link['indexes'][0]] = linked_thrusters
                to_pop.append(link['indexes'][1])
            for i in reversed(to_pop):
                thrusters.pop(i)
        
        self.thrusters = thrusters
        print([thruster.name for thruster in self.thrusters])

        self.max_thrust = max([t.max_thrust for t in self.thrusters])

    @staticmethod
    def calc_thrust_cancellation(thruster1 : Thruster, thruster2 : Thruster, power1 : float):
        force1, torque1 = thruster1.get_dynamics(power1)
        # Calculate the force unit vector for thruster21
        force_unit = np.array([np.sin(thruster2.rad_angle), np.cos(thruster2.rad_angle)])
        xy_cancel = np.array([force1[0] / force_unit[0], force1[1] / force_unit[1]])
        return xy_cancel / 4.44822
    
    def strafe_right(self):
        self.thrusters[0].set_normalized_thrust(1.0)
        self.thrusters[1].set_normalized_thrust(-1.0)
        self.thrusters[2].set_normalized_thrust(-1.0)
        self.thrusters[3].set_normalized_thrust(1.0)

    def full_thrust(self):
        for thruster in self.thrusters:
            thruster.set_normalized_thrust(1.0)

    def full_four_thrust(self):
        for i in range(4):
            self.thrusters[i].set_normalized_thrust(1.0)

    def update(self):
        net_force = np.array([0.0, 0.0])
        net_torque = 0.0
        
        for thruster in self.thrusters:
            force, torque = thruster.get_dynamics()
            net_force += force
            # assumes a constant moment of inertia
            net_torque += torque
        
        self.position += (net_force / self.max_thrust) * self.dt
        self.orientation += (net_torque / self.max_thrust) * self.dt
        print(f'Position: {self.position}, Orientation: {self.orientation}')
        print(f'Net Force: {net_force} Newtons, Net Torque: {net_torque} Newton-meters')


class simple_motion_sim:
    def __init__(self, config_file):
        self.boat = Boat(config_file)
        self.boat.strafe_right()

    def setup_plot(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        self.boat_patch = Rectangle(self.boat.position - 0.5, 1, 2, angle=0.0, fc='blue', rotation_point='center')
        self.ax.add_patch(self.boat_patch)

    def update(self, frame):
        self.boat.update()
        self.boat_patch.set_xy(self.boat.position - 0.5)
        self.boat_patch.angle = np.degrees(self.boat.orientation)
        return self.boat_patch,

    def run_simulation(self):
        self.setup_plot()
        ani = FuncAnimation(self.fig, self.update, frames=range(100), blit=True, interval=100)
        plt.show()


if __name__ == "__main__":
    sim = simple_motion_sim("configs/wamv_config.json")
    sim.run_simulation()
    res = Boat.calc_thrust_cancellation(sim.boat.thrusters[0], sim.boat.thrusters[3], 30.0)
    print(res)
    print(res[1] / res[0])