import cv2
import numpy as np
import math
import time

class Goal:
    def __init__(self, position : np.array, gain : float):
        self.position = position
        self.gain = gain

class Obstacle:
    def __init__(self, position : np.array, radius : float, gain : float):
        self.position = position
        self.radius = radius + 50
        self.gain = gain

def attractive_force(current_position : np.array, goal : Goal) -> np.array:
    force = goal.gain * (goal.position - current_position)
    magnitude = np.linalg.norm(force)
    direction = np.arctan2(force[1], force[0])
    if magnitude > 1.0:
        force = force / magnitude
    print("A Force:", force)
    return force, (magnitude, direction)

def repulsive_force(current_position : np.array, obstacle : Obstacle) -> np.array:
    force = np.zeros(2)
    distance = np.linalg.norm(current_position - obstacle.position)
    if distance < obstacle.radius:
        force = obstacle.gain * ((1.0 / distance) - (1.0 / obstacle.radius)) * (1.0 / (distance ** 2)) * (current_position - obstacle.radius)
    magnitude = np.linalg.norm(force)
    direction = np.arctan2(force[1], force[0])
    print("R Force:", force)
    return force, (magnitude, direction)


def potential_field(current_position : np.array, goal : Goal, obstacles : list) -> np.array:
    a_force, a_info = attractive_force(current_position, goal)
    r_force = np.zeros(2)
    for obstacle in obstacles:
        r_force += repulsive_force(current_position, obstacle)[0]
    force = a_force + r_force
    magnitude = np.linalg.norm(force)
    print("Force:", force)
    direction = np.arctan2(force[1], force[0])
    return force, (magnitude, direction)

# Define the obstacle map
MAP = np.zeros((300, 400, 3), np.uint8)

# Define the start and goal positions
start = np.array([0, 0], dtype=np.float32)
goal = Goal(np.array([350, 350], dtype=np.float32), 1)

# Define the obstacles
obstacles = []
obstacles.append(Obstacle(np.array([100, 75]), 20, 1000))
obstacles.append(Obstacle(np.array([200, 200]), 30, 1000))
obstacles.append(Obstacle(np.array([300, 300]), 40, 1000))
obstacles.append(Obstacle(np.array([400, 400]), 50, 1000))

while True:
    # Get the force and information
    force, info = potential_field(start, goal, obstacles)
    # print('Force:', force)
    # print('Magnitude:', info[0])
    # print('Direction:', info[1])

    # Update the position
    start += force

    # Draw the map
    map = MAP.copy()
    cv2.circle(map, (int(start[0]), int(start[1])), 5, (0, 0, 255), -1)
    cv2.circle(map, (int(goal.position[0]), int(goal.position[1])), 5, (0, 255, 0), -1)
    for obstacle in obstacles:
        cv2.circle(map, (int(obstacle.position[0]), int(obstacle.position[1])), int(obstacle.radius-60), (255, 255, 255), -1)
    
    cv2.imshow('Potential Field', map)

    # Check the keyboard input
    key = cv2.waitKey(1)

    # Check if the current position is the goal
    if np.linalg.norm(start - goal.position) < 1.0:
        print('Goal reached!')
        break

    # Check if the current position is in the obstacle
    for obstacle in obstacles:
        if np.linalg.norm(start - obstacle.position) < obstacle.radius:
            print('Obstacle detected!')
            break

    # Check if the current position is out of the map
    if start[0] < 0 or start[0] >= 400 or start[1] < 0 or start[1] >= 300:
        print('Out of the map!')
        break
    time.sleep(0.01)