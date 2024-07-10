import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

# Constants
dt = 0.1  # time step

# Initial state
boat_position = np.array([0.0, 0.0])  # x, y position
boat_orientation = 0.0  # angle in radians

# Thrusters (position relative to the boat's center)
thruster_positions = [ # the rectangle is horizontal
    np.array([0.5, 0.5]), # front left
    np.array([-0.5, 0.5]), # back left
    np.array([0.5, -0.5]), # front right
    np.array([-0.5, -0.5]) # back right
]

# Thrust levels and angles (initially zero)
thrust_levels = [0.0, 0.0, 0.0, 0.0]
thrust_angles = [0.0, 0.0, 0.0, 0.0]

# Update function
def update(frame):
    global boat_position, boat_orientation
    
    # Calculate forces and torques
    net_force = np.array([0.0, 0.0])
    net_torque = 0.0
    
    for i in range(4):
        thrust = thrust_levels[i]
        angle = thrust_angles[i] + boat_orientation
        force = thrust * np.array([np.cos(angle), np.sin(angle)])
        net_force += force
        torque = np.cross(thruster_positions[i], force)
        net_torque += torque
    
    # Update position and orientation
    boat_position += net_force * dt
    boat_orientation += net_torque * dt
    
    # Update plot
    boat.set_xy(boat_position - 0.5)
    boat.angle = np.degrees(boat_orientation)
    
    return boat,

# Set up the plot
fig, ax = plt.subplots()
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)

boat = Rectangle(boat_position - 0.5, 1, 1, angle=0.0, fc='blue')
ax.add_patch(boat)

# Animation
ani = FuncAnimation(fig, update, frames=range(100), blit=True, interval=100)
plt.show()
