import numpy as np
import cv2
import math
from queue import PriorityQueue

# Function to map coordinates to the bottom left of the image
def map_to_bottom_left(x, y, width, height):
    """
    Maps the given coordinates to the bottom left corner of a rectangle.

    Parameters:
    x (int): The x-coordinate of the point.
    y (int): The y-coordinate of the point.
    width (int): The width of the rectangle.
    height (int): The height of the rectangle.

    Returns:
    tuple: A tuple containing the x and y coordinates of the point mapped to the bottom left corner.
    """

    bottom_left_x = x
    bottom_left_y = height - y
    return bottom_left_x, bottom_left_y

# Function to draw a rectangle on the image
def draw_rectangle(img, bottom_left, top_right, color, thickness):
    """
    Draw a rectangle on the given image.

    Parameters:
    - img: The image on which the rectangle will be drawn.
    - bottom_left: The coordinates of the bottom left corner of the rectangle.
    - top_right: The coordinates of the top right corner of the rectangle.
    - color: The color of the rectangle.
    - thickness: The thickness of the rectangle's edges.

    Returns:
    None
    """
    cv2.rectangle(img, bottom_left, top_right, color, thickness)

# Function to draw a hexagon on the image
def draw_hexagon(img, center, side_length, color, thickness):
    """
    Draws a hexagon on the given image.

    Parameters:
    - img: The image on which the hexagon will be drawn.
    - center: The center coordinates of the hexagon.
    - side_length: The length of each side of the hexagon.
    - color: The color of the hexagon.
    - thickness: The thickness of the hexagon's outline.

    Returns:
    None
    """

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



# Function to ask for a point from the user
def ask_for_point(message, default=None):
    """
    Asks the user to input a point and validates its validity based on the obstacle map.

    Args:
        message (str): The message to display when asking for the point.
        default (tuple, optional): The default point to use if the user does not provide any input. 
                                   Defaults to None.

    Returns:
        tuple: The validated point (x, y).

    Raises:
        ValueError: If no default value is provided and the user does not provide any input.
    """
    while True:
        user_input = input(f"{message} (default: {default[0]},{default[1]}): ")
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
    """
    Calculates the Euclidean distance between two points in a two-dimensional space.
    
    Parameters:
        p1 (tuple): The coordinates of the first point (x1, y1).
        p2 (tuple): The coordinates of the second point (x2, y2).
        
    Returns:
        float: The Euclidean distance between the two points.
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Function to visualize the explored nodes
def visualize_explored(obstacle_map, explored_nodes):
    """
    Visualizes the explored nodes on the obstacle map.

    Args:
        obstacle_map (numpy.ndarray): The obstacle map to visualize on.
        explored_nodes (list): A list of explored nodes.

    Returns:
        None
    """

    count = 0
    cv2.circle(obstacle_map, (start[0], start[1]), 5, (255, 0, 0), -1)  # Explored nodes in green
    cv2.circle(obstacle_map, (goal[0], goal[1]), 5, (0, 0, 255), -1)  # Explored nodes in green

    for node in explored_nodes:
        count += 1
        obstacle_map[node[1], node[0]] = (0, 255, 0)
        if count % explored_interval == 0:
            cv2.imshow("Obstacle Map with Shortest Path", obstacle_map)
            out.write(obstacle_map)
            cv2.waitKey(1)

# Function to get neighboring points
def get_neighbors(point, obstacles):
    """
    Returns a list of valid neighboring points for a given point.

    Args:
        point (tuple): The coordinates of the point.
        obstacles (list): A list of coordinates representing obstacles.

    Returns:
        list: A list of valid neighboring points.

    """
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
    """
    Checks if a given point is a valid neighbor based on the obstacle map.

    Args:
        point (tuple): The coordinates of the point to check.
        obstacles (numpy.ndarray): The obstacle map.

    Returns:
        bool: True if the point is a valid neighbor, False otherwise.
    """
    x, y = point
    if 0 <= x < width and 0 <= y < height and obstacle_map[y, x, 0] == 255:
        return True
    return False

# Function to calculate the cost of moving from one point to another
def distance_cost(current, next):
    """
    Calculates the cost of moving from the current node to the next node.
    
    Parameters:
        current (tuple): The coordinates of the current node.
        next (tuple): The coordinates of the next node.
    
    Returns:
        float: The cost of moving from the current node to the next node.
    """
    if euclidean_distance(current, next) == 1.0:
        return 1.0
    else:
        return 1.4

# Dijkstra's algorithm for finding the shortest path
def dijkstra(start, goal, obstacles):
    """
    Finds the shortest path from the start node to the goal node using Dijkstra's algorithm.

    Args:
        start: The starting node.
        goal: The goal node.
        obstacles: A list of obstacles in the graph.

    Returns:
        A list representing the shortest path from the start node to the goal node.
    """
    
    frontier = PriorityQueue()
    frontier.put((0, start))

    cost_so_far = {start: 0}
    came_from = {start: None}
    explored_nodes = []  # List to store explored nodes

    while not frontier.empty():
        current_cost, current_node = frontier.get()

        if current_cost > cost_so_far.get(current_node, float('inf')):
            continue

        if current_node == goal:
            break

        for next in get_neighbors(current_node, obstacles):
            new_cost = cost_so_far[current_node] + distance_cost(current_node, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put((priority, next))
                came_from[next] = current_node
                explored_nodes.append(next)

    # Update visualization at intervals or when significant progress has been made
    visualize_explored(obstacle_map, explored_nodes)

    # Trace back the shortest path
    path = []
    current_node = goal
    while current_node != start:
        path.append(current_node)
        current_node = came_from[current_node]
    path.append(start)
    path.reverse()

    return path


# Define image dimensions
width = 1200
height = 500
explored_interval = 200  # Update visualization every 200 explored nodes

# Create a blank image filled with white
obstacle_map = np.ones((height, width, 3), dtype=np.uint8) * 255

# Define obstacles
obstacles = [
    {'shape': 'rectangle', 'bottom_left': (95, 95), 'top_right': (180, 500), 'color': (128, 128, 128), 'thickness': -1},  # Rectangle obstacle 1
    {'shape': 'rectangle', 'bottom_left': (100, 100), 'top_right': (175, 500), 'color': (0, 0, 0), 'thickness': -1},  # Rectangle obstacle 2
    {'shape': 'rectangle', 'bottom_left': (270, 0), 'top_right': (355, 405), 'color': (128, 128, 128), 'thickness': -1},  # Rectangle obstacle 3
    {'shape': 'rectangle', 'bottom_left': (275, 0), 'top_right': (350, 400), 'color': (0, 0, 0), 'thickness': -1},  # Rectangle obstacle 4
    {'shape': 'hexagon', 'center': (650, 250), 'side_length': 150, 'color': (128, 128, 128), 'thickness': 4},  # Hexagon obstacle
    {'shape': 'rectangle', 'bottom_left': (1020, 45), 'top_right': (1105, 455), 'thickness': -1, 'color': (128, 128, 128)},  # Rectangle obstacle 5
    {'shape': 'rectangle', 'bottom_left': (895, 45), 'top_right': (1105, 130), 'thickness': -1, 'color': (128, 128, 128)},  # Rectangle obstacle 6
    {'shape': 'rectangle', 'bottom_left': (900, 50), 'top_right': (1100, 125), 'thickness': -1, 'color': (0, 0, 0)},  # Rectangle obstacle 7
    {'shape': 'rectangle', 'bottom_left': (895, 370), 'top_right': (1105, 455), 'thickness': -1, 'color': (128, 128, 128)},  # Rectangle obstacle 8
    {'shape': 'rectangle', 'bottom_left': (900, 375), 'top_right': (1100, 450), 'thickness': -1, 'color': (0, 0, 0)},  # Rectangle obstacle 9
    {'shape': 'rectangle', 'bottom_left': (1025, 50), 'top_right': (1100, 450), 'thickness': -1, 'color': (0, 0, 0)},  # Rectangle obstacle 10
    {'shape': 'rectangle', 'bottom_left': (0, 0), 'top_right': (1200, 5), 'color': (128, 128, 128), 'thickness': -1},  # Rectangle obstacle 11
    {'shape': 'rectangle', 'bottom_left': (0, 0), 'top_right': (5, 500), 'color': (128, 128, 128), 'thickness': -1},  # Rectangle obstacle 12
    {'shape': 'rectangle', 'bottom_left': (1195, 0), 'top_right': (1200, 500), 'color': (128, 128, 128), 'thickness': -1},  # Rectangle obstacle 13
    {'shape': 'rectangle', 'bottom_left': (0, 495), 'top_right': (1200, 500), 'color': (128, 128, 128), 'thickness': -1},  # Rectangle obstacle 14
]


# Draw obstacles on the obstacle map
draw_obstacles(obstacle_map, obstacles)

# Ask for start and end points
start = ask_for_point("Enter start point (x, y): ", (50, 50))
goal = ask_for_point("Enter goal point (x, y): ", (1150, 50))

print("Start point:", start)
print("Goal point:", goal)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# Save the obstacle map with the shortest path as a video
out = cv2.VideoWriter('obstacle_map_with_shortest_path.mp4', fourcc, 60.0, (width, height))


# Find the shortest path using Dijkstra's algorithm
shortest_path = dijkstra(start, goal, obstacles)

# Mark the shortest path on the obstacle map
for point in shortest_path:
    cv2.circle(obstacle_map, (point[0], point[1]), 2, (255, 0, 0), -1)  # Explored nodes in green
    cv2.imshow("Obstacle Map with Shortest Path", obstacle_map)
    out.write(obstacle_map)
    cv2.waitKey(1)

# Display the obstacle map with the shortest path
cv2.imshow("Obstacle Map with Shortest Path", obstacle_map)
out.write(obstacle_map)

cv2.waitKey(0)
out.release()
cv2.destroyAllWindows()
