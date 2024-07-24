import cv2
import numpy as np
import math
import time
from typing import List

class Goal:
    def __init__(self, position : np.array, gain : float, radius : float = 10):
        self.position = position
        self.radius = radius
        self.gain = gain

class Obstacle:
    def __init__(self, position : np.array, object_radius : float, gain : float = 200000):
        self.position = position
        self.radius = object_radius*2.5
        self.gain = gain
        self.draw_radius = int(object_radius)

def attractive_force(current_position : np.array, goal : Goal) -> np.array:
    force = goal.gain * (goal.position - current_position)
    magnitude = np.linalg.norm(force)
    direction = np.arctan2(force[1], force[0])
    return force, (magnitude, direction)

def repulsive_force(current_position : np.array, obstacle : Obstacle) -> np.array:
    force = np.zeros(2)
    distance = np.linalg.norm(current_position - obstacle.position)
    if distance < obstacle.radius:
        force = obstacle.gain * ((1.0 / distance) - (1.0 / obstacle.radius)) * (1.0 / (distance ** 2)) * (current_position - obstacle.position)
    magnitude = np.linalg.norm(force)
    direction = np.arctan2(force[1], force[0])
    return force, (magnitude, direction)

def potential_field(current_position : np.array, goals : List[Goal], obstacles : List[Obstacle]) -> np.array:
    # Attractive force
    a_force = np.zeros(2)
    closest_goal = min(goals, key=lambda goal: np.linalg.norm(current_position - goal.position), default=None)
    if closest_goal is not None:
        a_force += attractive_force(current_position, closest_goal)[0]
    
    # Repulsive Force
    r_force = np.zeros(2)
    for obstacle in obstacles:
        r_force += repulsive_force(current_position, obstacle)[0]
    force = a_force + r_force

    # Total force
    magnitude = np.linalg.norm(force)
    direction = np.arctan2(force[1], force[0])
    if magnitude > 1.0:
        force = force / magnitude
    return force, (magnitude, direction)

# Define the obstacle map
map_size = (720, 1280)
MAP = np.zeros((map_size[0], map_size[1], 3), np.uint8)

# Define the start and end goal positions
start = np.array([0, 0], dtype=np.float32)
end_goal = Goal(np.array([450, 450], dtype=np.float32), 0.1)

# Define the goals
goals: List[Goal] = [
    Goal(np.array([200, 75], dtype=np.float32), 0.1),
    Goal(np.array([150, 125], dtype=np.float32), 0.1),
    end_goal
]

# Define the obstacles
obstacles: List[Obstacle] = [
    Obstacle(np.array([100, 75]), 20),
    Obstacle(np.array([200, 200]), 30),
    Obstacle(np.array([300, 300]), 40)
]

while True:
    # Get the force and information
    force, info = potential_field(start, goals, obstacles)

    # Update the position
    start += force

    # Draw the map
    map = MAP.copy()
    cv2.circle(map, (int(start[0]), int(start[1])), 5, (0, 0, 255), -1)
    for goal in goals:
        cv2.circle(map, (int(goal.position[0]), int(goal.position[1])), 5, (0, 255, 0), -1)
    for obstacle in obstacles:
        cv2.circle(map, (int(obstacle.position[0]), int(obstacle.position[1])), int(obstacle.draw_radius), (255, 255, 255), -1)
        cv2.circle(map, (int(obstacle.position[0]), int(obstacle.position[1])), int(obstacle.radius), (0, 0, 255), 1)
    
    cv2.imshow('Potential Field', map)

    # Check the keyboard input
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

    # Check if the current position is the goal
    for i in range(len(goals)-1, -1, -1):
        if np.linalg.norm(start - goals[i].position) < goals[i].radius:
            print('Goal reached!')
            goals.pop(i)
            break

    if len(goals) == 0:
        print('All goals reached!')
        break
    
    # Check if the current position is in the obstacle
    for obstacle in obstacles:
        if np.linalg.norm(start - obstacle.position) < obstacle.draw_radius:
            print('Hit Obstacle!')
            break

    # Check if the current position is out of the map
    if start[0] < 0 or start[0] >= map_size[0] or start[1] < 0 or start[1] >= map_size[1]:
        print('Out of the map!')
        break
    time.sleep(0.01)