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

def draw_obstacles(obstacle_map, obstacles):
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
# Define obstacles
obstacles = [
    {'shape': 'rectangle', 'bottom_left': (100, 100), 'top_right': (175, 500), 'color': (0, 0, 0), 'thickness': -1},  

    {'shape': 'rectangle', 'bottom_left': (275, 0), 'top_right': (350, 400), 'color': (0, 0, 0), 'thickness':-1},

    {'shape': 'hexagon', 'center': (650, 250), 'side_length': 150,'color': (128, 128, 128), 'thickness': 4},  


    {'shape': 'rectangle', 'bottom_left': (900, 50), 'top_right': (1100, 125), 'thickness': -1,'color': (0,0, 0)}, 

    {'shape': 'rectangle', 'bottom_left': (900, 375), 'top_right': (1100, 450), 'thickness': -1,'color': (0, 0, 0)}, 

    {'shape': 'rectangle', 'bottom_left': (1025, 50), 'top_right': (1100, 450), 'thickness': -1,'color': (0, 0, 0)}, 


]

# Draw obstacles on the obstacle map
draw_obstacles(obstacle_map, obstacles)




# Display the obstacle map
cv2.imshow("Obstacle Map", obstacle_map)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Now you can use obstacle_map for your navigation algorithm with your point robot
