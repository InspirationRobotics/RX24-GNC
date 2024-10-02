import cv2
import numpy as np
import math

# Define the obstacle map
MAP = np.zeros((300, 400, 3), np.uint8)
cv2.rectangle(MAP, (50, 50), (70, 70), (255, 255, 255), -1)
cv2.rectangle(MAP, (200, 50), (250, 200), (255, 255, 255), -1)
cv2.rectangle(MAP, (150, 200), (250, 250), (255, 255, 255), -1)
cv2.rectangle(MAP, (50, 250), (100, 300), (255, 255, 255), -1)

# Define the start and goal positions
start = (0, 100)
goal = (350, 250)

# Define the motion model
motion = [[5, 0],
          [0, 5],
          [-5, 0],
          [0, -5],
          [-5, -5],
          [-5, 5],
          [5, -5],
          [5, 5]]

# Define the cost of each motion // diagonal cost is sqrt(2)
cost = [1, 1, 1, 1, math.sqrt(2), math.sqrt(2), math.sqrt(2), math.sqrt(2)]

# Define the heuristic function // just solves for the distance between two points
def heuristic(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

# Define the search function
def search():
    # Initialize the node list
    open_list = []
    open_list.append([0, start[0], start[1]])

    # Initialize the cost map
    cost_map = np.zeros((300, 400))
    cost_map[:] = np.inf
    cost_map[start[1], start[0]] = 0

    # Initialize the action map
    action_map = np.zeros((300, 400), np.uint8)
    action_map[:] = 0

    # Search loop
    while open_list:
        # Get the node with the lowest cost
        current_node = open_list.pop(0)
        x, y = current_node[1], current_node[2]

        # Check if the current node is the goal
        if x == goal[0] and y == goal[1]:
            break

        # Expand the current node
        for i in range(len(motion)):
            x2, y2 = x + motion[i][0], y + motion[i][1]

            # Check if the new node is within the map
            if x2 >= 0 and x2 < 400 and y2 >= 0 and y2 < 300:
                # Check if the new node is not an obstacle
                if MAP[y2, x2, 0] != 255:
                    # Calculate the new cost
                    new_cost = cost[i] + cost_map[y, x]

                    # Check if the new cost is lower than the current cost
                    if new_cost < cost_map[y2, x2]:
                        # Update the cost and action maps
                        cost_map[y2, x2] = new_cost
                        action_map[y2, x2] = i

                        # Calculate the heuristic cost
                        h = heuristic([x2, y2], goal)

                        # Add the new node to the open list
                        open_list.append([new_cost + h, x2, y2])

                        # Sort the open list
                        open_list = sorted(open_list)

    # Extract the path
    path = []
    x, y = goal[0], goal[1]
    while x != start[0] or y != start[1]:
        i = action_map[y, x]
        path.append([x, y])
        x -= motion[i][0]
        y -= motion[i][1]
    path.append([x, y])

    return path

# Search for the path
path = search()

# Draw the path
for i in range(len(path) - 1):
    cv2.line(MAP, (path[i][0], path[i][1]), (path[i + 1][0], path[i + 1][1]), (0, 0, 255), 2)

# Display the map
cv2.imshow('Map', MAP)
cv2.waitKey(0)
cv2.destroyAllWindows()