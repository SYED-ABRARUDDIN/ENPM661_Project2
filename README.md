


# Dijkstra's Algorithm for Shortest Path ENMP661 PROJECT2
**Author: Abraruddin Syed**  
**UID: 120109997**


This Python script implements Dijkstra's algorithm to find the shortest path between a start point and a goal point on an obstacle map. The obstacle map is a 2D image where obstacles are represented by filled shapes (rectangles and hexagons).

## Prerequisites

- Python 3.x
- OpenCV (cv2) library
- NumPy library

## Usage

1. Clone the repository or download the `dijkstra_Abraruddin_Syed.py` file.
2. Install the required libraries using pip:
   ```
   pip install opencv-python numpy
   ```
3. Run the script:
   ```
   python dijkstra_Abraruddin_Syed.py
   ```
4. Follow the prompts to input the start and goal points.

### Example Input

After running the script, you will be prompted to input the start and goal points. Here's an example of how you might provide this input:

```
Enter start point (x, y):  (default: 50,50): 50, 450
Enter goal point (x, y):  (default: 1150,50): 50, 1150
```

In this example, the start point is at coordinates (50, 50) and the goal point is at coordinates (50,1150). Please note that the coordinates should be within the dimensions of your obstacle map.


You can add this section to your README file to provide users with an example of how to input the start and goal points.
5. The script will display the obstacle map and explored nodes with the shortest path highlighted in blue.


## Customization

- You can modify the obstacle map by changing the dimensions, adding or removing obstacles, or adjusting their properties (shape, color, thickness, transparency).
- The visualization of explored nodes can be customized by changing the `explored_interval` variable to control the frequency of updates.



## Acknowledgements

- The script is based on Dijkstra's algorithm for pathfinding.
- The visualization is done using the OpenCV library.

## Results


