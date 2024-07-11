import json
import numpy as np
from typing import List
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch
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

    def get_dynamics(self, *, normalized_thrust : float = None, raw_thrust : float = None):
        if normalized_thrust is not None:
            thrust = normalized_thrust * self.max_thrust
            thrust = np.clip(thrust, -self.max_thrust, self.max_thrust)
        elif raw_thrust is not None:
            thrust = raw_thrust
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

    def get_dynamics(self, *, normalized_thrust : float = None, raw_thrust : float = None):
        if normalized_thrust is not None:
            thrust = normalized_thrust * self.max_thrust
            thrust = np.clip(thrust, -self.max_thrust, self.max_thrust)
        elif raw_thrust is not None:
            thrust = raw_thrust
        else:
            thrust = self.thrust
        force1, torque1 = self.thruster1.get_dynamics(raw_thrust = thrust * self.thrust1_ratio)
        force2, torque2 = self.thruster2.get_dynamics(raw_thrust = thrust * self.thrust2_ratio)
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
        self.names = [thruster.name for thruster in self.thrusters]
        print(self.names)

        self.max_thrust = max([t.max_thrust for t in self.thrusters])

    @staticmethod
    def calc_thrust_cancellation(thruster1 : Thruster, thruster2 : Thruster, power1 : float):
        force1, _ = thruster1.get_dynamics(normalized_thrust=power1)
        # Calculate the force unit vector for thruster2
        force_unit = np.array([np.sin(thruster2.rad_angle), np.cos(thruster2.rad_angle)])
        print(force1, force_unit)
        xy_cancel = np.array([force1[0] / force_unit[0], force1[1] / force_unit[1]])
        return (xy_cancel / 4.44822) / thruster2.max_thrust
    
    def strafe_left(self):
        self.thrusters[0].set_normalized_thrust(1.0) # front right
        self.thrusters[1].set_normalized_thrust(-1.0) # front left
        self.thrusters[2].set_normalized_thrust(-1.0) # back right
        self.thrusters[3].set_normalized_thrust(1.0) # back left

    def strafe_wamv_left(self):
        self.thrusters[0].set_normalized_thrust(0.2) # front right
        self.thrusters[1].set_normalized_thrust(-0.2) # front left
        self.thrusters[2].set_normalized_thrust(-0.138) # back right
        self.thrusters[3].set_normalized_thrust(0.138) # back left

    def strafe_wamv_right(self):
        self.thrusters[0].set_normalized_thrust(-0.2) # front right
        self.thrusters[1].set_normalized_thrust(0.2) # front left
        self.thrusters[2].set_normalized_thrust(0.138) # back right
        self.thrusters[3].set_normalized_thrust(-0.138) # back left

    def rotate_wamv_ccw(self):
        self.thrusters[0].set_normalized_thrust(0.2)
        self.thrusters[1].set_normalized_thrust(-0.2)
        self.thrusters[2].set_normalized_thrust(0.138)
        self.thrusters[3].set_normalized_thrust(-0.138)

    def rotate_ccw(self):
        self.thrusters[0].set_normalized_thrust(0.2)
        self.thrusters[1].set_normalized_thrust(-0.2)
        self.thrusters[2].set_normalized_thrust(0.2)
        self.thrusters[3].set_normalized_thrust(-0.2)

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
        
        # Lets assume a drag coefficient of 0.2 for translational
        net_force *= 0.8
        # Lets assume a drag coefficient of 0.8 for rotational
        net_torque *= 0.2
        self.position += (net_force / self.max_thrust) * self.dt
        self.orientation += (net_torque / self.max_thrust) * self.dt
        print("==================================")
        print(self.names)
        print(f'Position: {self.position}, Orientation: {self.orientation}')
        print(f'Net Force: {net_force} Newtons, Net Torque: {net_torque} Newton-meters')


class simple_motion_sim:
    def __init__(self, config_file):
        self.boat = Boat(config_file)
        self.boat.rotate_wamv_ccw()

    def calc_arrow(self, arrow : FancyArrowPatch, position : List[float], orientation : float):
        orientation = orientation + np.pi / 2
        tail = [position[0], position[1]]
        head = [position[0] + np.cos(orientation)*5, position[1] + np.sin(orientation)*5]
        arrow.set_positions(tail, head)

    def setup_plot(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        self.boat_patch = Rectangle(self.boat.position - 0.5, 1, 2, angle=0.0, fc='blue', rotation_point='center')
        self.arrow = FancyArrowPatch(self.boat.position, self.boat.position, arrowstyle='->', mutation_scale=10, color='red')
        self.calc_arrow(self.arrow, self.boat.position, self.boat.orientation)
        self.ax.add_patch(self.boat_patch)
        self.ax.add_patch(self.arrow)

    def update(self, frame):
        self.boat.update()
        self.boat_patch.set_xy(self.boat.position - 0.5)
        self.boat_patch.angle = np.degrees(self.boat.orientation)
        # Draw an arrow for the orientation across the rectangle
        self.calc_arrow(self.arrow, self.boat.position, self.boat.orientation)
        # self.arrow.set_positions(self.boat.position, self.boat.position + 0.5 * np.array([np.cos(self.boat.orientation + np.pi / 2), np.sin(self.boat.orientation + np.pi / 2)]))

        return self.boat_patch, self.arrow,

    def run_simulation(self):
        self.setup_plot()
        ani = FuncAnimation(self.fig, self.update, frames=range(100), blit=True, interval=100)
        plt.show()


if __name__ == "__main__":
    sim = simple_motion_sim("configs/wamv_config.json")
    sim.run_simulation()
    res = Boat.calc_thrust_cancellation(sim.boat.thrusters[0], sim.boat.thrusters[2], 0.2)
    print(res)