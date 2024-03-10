import numpy as np
import cv2
import math
from queue import PriorityQueue

# Function to map coordinates to the bottom left of the image
def map_to_bottom_left(x, y, width, height):
    bottom_left_x = x
    bottom_left_y = height - y
    return bottom_left_x, bottom_left_y

# Function to draw a rectangle on the image
def draw_rectangle(img, bottom_left, top_right, color, thickness):
    cv2.rectangle(img, bottom_left, top_right, color, thickness)

# Function to draw a hexagon on the image
def draw_hexagon(img, center, side_length, color, thickness):
    # Calculate the coordinates of the hexagon's vertices
    vertices = []
    for i in range(6):
        angle_deg = 60 * i - 30
        angle_rad = math.pi / 180 * angle_deg
        x = center[0] + side_length * math.cos(angle_rad)
        y = center[1] + side_length * math.sin(angle_rad)
        vertices.append((int(x), int(y)))

    # Draw the hexagon filled with the specified color
    cv2.fillPoly(img, [np.array(vertices)], (0, 0, 0))
    cv2.polylines(img, [np.array(vertices)], isClosed=True, color=color, thickness=thickness)

def draw_obstacles(obstacle_map, obstacles):
    """
    Draw obstacles on the obstacle map.

    Args:
        obstacle_map (numpy.ndarray): The obstacle map.
            A 2D numpy array representing the obstacle map.
        obstacles (list): List of dictionaries representing the obstacles.
            Each dictionary in the list represents an obstacle and contains the following keys:
            - 'shape' (str): The shape of the obstacle ('rectangle' or 'hexagon').
            - 'color' (tuple, optional): The color of the obstacle in RGB format. Default is black (0, 0, 0).
            - 'thickness' (int, optional): The thickness of the obstacle's outline. Default is -1 (filled shape).
            - 'transparency' (float, optional): The transparency of the obstacle. Default is 1.0 (fully opaque).
            - 'bottom_left' (tuple): The bottom-left coordinates of the obstacle.
            - 'top_right' (tuple): The top-right coordinates of the obstacle.
            - 'center' (tuple): The center coordinates of the obstacle (for hexagon shape).
            - 'side_length' (int): The side length of the hexagon (for hexagon shape).

    Returns:
        None
    """
    for obstacle in obstacles:
        color = obstacle.get('color', (0, 0, 0))  # Default color is black
        thickness = obstacle.get('thickness', -1)  # Default to filled if thickness is not specified
        transparency = obstacle.get('transparency', 1.0)  # Default transparency is fully opaque
        color_with_transparency = tuple(list(color) + [transparency * 255])
        
        if obstacle['shape'] == 'rectangle':
            bottom_left = map_to_bottom_left(obstacle['bottom_left'][0], obstacle['bottom_left'][1], obstacle_map.shape[1], obstacle_map.shape[0])
            top_right = map_to_bottom_left(obstacle['top_right'][0], obstacle['top_right'][1], obstacle_map.shape[1], obstacle_map.shape[0])
            draw_rectangle(obstacle_map, bottom_left, top_right, color_with_transparency, thickness)
        elif obstacle['shape'] == 'hexagon':
            center = map_to_bottom_left(obstacle['center'][0], obstacle['center'][1], obstacle_map.shape[1], obstacle_map.shape[0])
            draw_hexagon(obstacle_map, center, obstacle['side_length'], color_with_transparency, thickness)


# Define image dimensions
width = 1200
height = 500

# Create a blank image filled with white
obstacle_map = np.ones((height, width, 3), dtype=np.uint8) * 255


# Define obstacles
obstacles = [
    {'shape': 'rectangle', 'bottom_left': (95, 95), 'top_right': (180, 500), 'color': (128, 128, 128), 'thickness':-1},  # Rectangle obstacle 1
    {'shape': 'rectangle', 'bottom_left': (100, 100), 'top_right': (175, 500), 'color': (0, 0, 0), 'thickness': -1},  # Rectangle obstacle 2

    {'shape': 'rectangle', 'bottom_left': (270, 0), 'top_right': (355, 405), 'color': (128, 128, 128), 'thickness':-1},  # Rectangle obstacle 3
    {'shape': 'rectangle', 'bottom_left': (275, 0), 'top_right': (350, 400), 'color': (0, 0, 0), 'thickness':-1},  # Rectangle obstacle 4

    {'shape': 'hexagon', 'center': (650, 250), 'side_length': 150,'color': (128, 128, 128), 'thickness': 4},  # Hexagon obstacle

    {'shape': 'rectangle', 'bottom_left': (1020, 45), 'top_right': (1105, 455), 'thickness': -1, 'color': (128, 128, 128)},  # Rectangle obstacle 5

    {'shape': 'rectangle', 'bottom_left': (895, 45), 'top_right': (1105, 130), 'thickness': -1, 'color': (128, 128, 128)},  # Rectangle obstacle 6
    {'shape': 'rectangle', 'bottom_left': (900, 50), 'top_right': (1100, 125), 'thickness': -1,'color': (0,0, 0)},  # Rectangle obstacle 7

    {'shape': 'rectangle', 'bottom_left': (895, 370), 'top_right': (1105, 455), 'thickness': -1, 'color': (128, 128, 128)},  # Rectangle obstacle 8
    {'shape': 'rectangle', 'bottom_left': (900, 375), 'top_right': (1100, 450), 'thickness': -1,'color': (0, 0, 0)},  # Rectangle obstacle 9

    {'shape': 'rectangle', 'bottom_left': (1025, 50), 'top_right': (1100, 450), 'thickness': -1,'color': (0, 0, 0)},  # Rectangle obstacle 10

    {'shape': 'rectangle', 'bottom_left': (0, 0), 'top_right': (1200, 5), 'color': (128, 128, 128), 'thickness': -1},  # Rectangle obstacle 11
    {'shape': 'rectangle', 'bottom_left': (0, 0), 'top_right': (5, 500), 'color': (128, 128, 128), 'thickness': -1},  # Rectangle obstacle 12
    {'shape': 'rectangle', 'bottom_left': (1195, 0), 'top_right': (1200, 500), 'color': (128, 128, 128), 'thickness': -1},  # Rectangle obstacle 13
    {'shape': 'rectangle', 'bottom_left': (0, 495), 'top_right': (1200, 500), 'color': (128, 128, 128), 'thickness': -1},  # Rectangle obstacle 14
]

# Draw obstacles on the obstacle map
draw_obstacles(obstacle_map, obstacles)

# Function to ask for a point from the user
def ask_for_point(message, default=None):
    while True:
        user_input =  input(f"{message} (default: {default[0]},{default[1]}): ")
        if user_input.strip() == "":
            if default is None:
                raise ValueError("No default value provided.")
            else:
                x, y = default
                x, y = map_to_bottom_left(x, y, width, height)
        else:
            x, y = map(int, user_input.split(','))
            x, y = map_to_bottom_left(x, y, width, height)

        if 0 <= x < width and 0 <= y < height and obstacle_map[y, x, 0] == 255:
            return x, y
        else:
            print("Point is invalid.")

# Function to calculate the Euclidean distance between two points
def euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Dijkstra's algorithm for finding the shortest path
def dijkstra(start, goal, obstacles):
    frontier = PriorityQueue()
    frontier.put(start, 0)

    cost_so_far = {start: 0}
    came_from = {start: None}

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in get_neighbors(current, obstacles):
            new_cost = cost_so_far[current] + distance_cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current

    # Trace back the shortest path
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path

# Function to get neighboring points
def get_neighbors(point, obstacles):
    x, y = point
    neighbors = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            neighbor = (x + dx, y + dy)
            if is_valid_neighbor(neighbor, obstacles):
                neighbors.append(neighbor)
    return neighbors

# Function to check if a point is a valid neighbor
def is_valid_neighbor(point, obstacles):
    x, y = point
    if 0 <= x < width and 0 <= y < height and obstacle_map[y, x,0] == 255:
        return True
    return False

# Function to calculate the cost of moving from one point to another
def distance_cost(current, next):
    dx = abs(current[0] - next[0])
    dy = abs(current[1] - next[1])
    if dx == 0 or dy == 0:
        return 1
    else:
        return 1.4

# Ask for start and end points
start = ask_for_point("Enter start point (x, y): ", (50, 50))
goal = ask_for_point("Enter goal point (x, y): ", (1150, 50))

print("Start point:", start)
print("Goal point:", goal)

# Find the shortest path using Dijkstra's algorithm
shortest_path = dijkstra(start, goal, obstacles)

# Mark the shortest path on the obstacle map
for point in shortest_path:
    cv2.circle(obstacle_map, (point[0], point[1]), 2, (0, 255, 0), -1)  # Explored nodes in green
    cv2.imshow("Obstacle Map with Shortest Path", obstacle_map)
    cv2.waitKey(10)

# Display the obstacle map with the shortest path
cv2.imshow("Obstacle Map with Shortest Path", obstacle_map)
cv2.waitKey(0)

cv2.destroyAllWindows()
# Now you can use obstacle_map for your navigation algorithm with your point robot
