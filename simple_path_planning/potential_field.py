import cv2
import numpy as np
import math
import time
from typing import List

class Goal:
    def __init__(self, position : np.array, gain : float, radius : float = 10):
        if isinstance(position, list):
            position = np.array(position)
        self.position : np.ndarray = position
        self.radius = radius
        self.gain = gain

class Obstacle:
    def __init__(self, position : np.array, object_radius : float, gain : float = 10000):
        self.position = position
        self.radius = object_radius*2
        self.gain = gain
        self.draw_radius = int(object_radius)

def attractive_force(current_position : np.array, goal : Goal) -> np.array:
    force = goal.gain * (goal.position - current_position)
    magnitude = np.linalg.norm(force)
    direction = np.arctan2(force[1], force[0])
    return force, (magnitude, direction)

def sigmoid(x, k=10):
    return 1 / (1 + np.exp(-k * (x - 0.5)))

def repulsive_force(current_position : np.array, obstacle : Obstacle) -> np.array:
    force = np.zeros(2)
    distance = np.linalg.norm(current_position - obstacle.position)
    if distance < obstacle.radius:
        influence = sigmoid(1-(distance / obstacle.radius)) ** 2
        force = obstacle.gain * influence * (1.0 / (distance)) * (current_position - obstacle.position)
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
        force = 2 * (force / magnitude)

    # add a small random component to the force
    force += 0.1 * np.random.randn(2)
    return force, (magnitude, direction)

# Define the obstacle map
map_size = (720, 1280)
MAP = np.zeros((map_size[0], map_size[1], 3), np.uint8)

# Define the start and end goal positions
start = np.array([0, 360], dtype=np.float32)
end_goal = Goal(np.array([1000, 360], dtype=np.float32), 0.1)

# goal points:
def temp_goal_points(start_pos, end_pos, amount, *, variation=15):
    if isinstance(start_pos, np.ndarray):
        start_pos = start_pos.copy()
    if isinstance(end_pos, np.ndarray):
        end_pos = end_pos.copy()
    start_pos[0]+=40
    end_pos[0]-=40
    spacing = (end_pos[0] - start_pos[0]) / amount
    goal_points = []
    for i in range(amount):
        goal_points.append(Goal([start_pos[0], start_pos[1]], 0.1))
        start_pos[0]+=spacing
        start_pos[1]+=np.random.randint(-variation,variation)
    return goal_points

# obstacle points:
def temp_obstacle_points(goal_points : List[Goal]):
    obstacle_points = []
    for point in goal_points:
        pos1 = point.position.copy()
        pos2 = point.position.copy()
        pos1[1]-=20
        pos2[1]+=20
        obstacle_points.append(Obstacle(pos1, 5))
        obstacle_points.append(Obstacle(pos2, 5))
    return obstacle_points

# Define the goals
goals: List[Goal] = [
    end_goal
]

goal_points = temp_goal_points(start, end_goal.position, 20, variation=40)
goals.extend(goal_points)

# Define the obstacles
obstacles: List[Obstacle] = [
    # Obstacle([300, 350], 20)
]
obstacles.extend(temp_obstacle_points(goal_points))

while True:
    # Get the force and information
    force, info = potential_field(start, goals, obstacles)

    # Update the position
    start += force

    # Draw the map
    map = MAP.copy()
    cv2.circle(map, (int(start[0]), int(start[1])), 5, (0, 0, 255), -1)
    # draw an arrow showing the vector with direction and magnitude
    cv2.arrowedLine(map, (int(start[0]), int(start[1])), (int(start[0]+force[0]*info[0]), int(start[1]+force[1]*info[0])), (0, 0, 255), 2)


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

    if key == ord(' '):
        cv2.waitKey(0)

    # Check if the current position is the goal
    for i in range(len(goals)-1, -1, -1):
        if np.linalg.norm(start - goals[i].position) < goals[i].radius:
            # print('Goal reached!')
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
    if start[0] < 0 or start[0] >= map_size[1] or start[1] < 0 or start[1] >= map_size[0]:
        print(start)
        print('Out of the map!')
        break
    time.sleep(0.01)