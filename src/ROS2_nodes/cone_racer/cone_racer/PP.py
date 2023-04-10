import pickle, numpy as np, matplotlib.pyplot as plt, matplotlib.image as mpimg, cv2, time
from scipy.spatial.distance import cdist
from scipy.spatial import ConvexHull, Delaunay
from scipy.interpolate import splprep, splev
from shapely.geometry import MultiLineString
from shapely.ops import unary_union, polygonize
from collections import Counter
import itertools
from concave_hull import concave_hull

def extract_cone_centroids(map_data, resolution=0.05, xorigin=0, yorigin=0, threshold=42, white=101) -> np.ndarray:
    """
    Returns the centroids of the cones out of a SLAM map (gray-map occupancy grid)
    """
    map_data = cv2.threshold(map_data.astype(np.uint8), threshold, white, cv2.THRESH_BINARY)[1]

    contours, hierarchy = cv2.findContours(map_data, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[1:]

    centroids = [(cv2.minEnclosingCircle(cnt)[0][0] * resolution + xorigin,
                  cv2.minEnclosingCircle(cnt)[0][1] * resolution + yorigin) for cnt in contours]

    return np.array(centroids)

def nnb_sorting(points: np.ndarray, start_point=[0, 0]) -> np.ndarray:
    """
    Returns an the points sorted by nearest neighbour.
    """
    ordered_points = np.array([], dtype=np.float64).reshape(0, 2)
    current_point = start_point
    while points.shape[0] > 0:
        distances = cdist([current_point], points)[0]                       # Calculating distances to all other points
        min_distance_index = np.argmin(distances)                           # Choosing the closest point as the next point in the sequence
        current_point = points[min_distance_index]                          # Setting the next point as the current point
        points = np.delete(points, min_distance_index, axis=0)              # Deleting the visited point from the list of points
        ordered_points = np.append(ordered_points, [current_point], axis=0) # Adding the visited point to the ordered list

    return ordered_points

def find_lr_indices(robot_position, robot_direction, cone_positions):
    """
    Returns the indices of the cones on the left and right side of the robot based on the robot's position and direction.
    """
    robot_direction -= np.pi / 2
    robot_direction = np.array([np.cos(robot_direction), np.sin(robot_direction)]) # Convert robot direction to unit vector

    # Find the cones on the left side of the robot
    left_cone_indices, right_cone_indices = np.array([], dtype=np.int32), np.array([], dtype=np.int32)
    for i, cone_position in enumerate(cone_positions):
        cone_direction = cone_position - robot_position
        angle = np.arccos(np.dot(robot_direction, cone_direction) / (np.linalg.norm(robot_direction) * np.linalg.norm(cone_direction)))
        if angle > np.pi / 2.0: # cone is on the left side of the robot
            left_cone_indices = np.append(left_cone_indices, i)
        else:
            right_cone_indices = np.append(right_cone_indices, i)

    return left_cone_indices, right_cone_indices

def concave_hull(coords:np.ndarray, alpha:float): 
    """
    Return outer hull of a set of points allowing concavity (alpha=0 is convex hull).
    """
    tri = Delaunay(coords, qhull_options="Qc Qz Q12").vertices

    ia, ib, ic = (          # indices of each of the triangles' points
        tri[:, 0],
        tri[:, 1],
        tri[:, 2],
    )  
    pa, pb, pc = (          # coordinates of each of the triangles' points
        coords[ia],
        coords[ib],
        coords[ic],
    )  

    a = np.sqrt((pa[:, 0] - pb[:, 0]) ** 2 + (pa[:, 1] - pb[:, 1]) ** 2)
    b = np.sqrt((pb[:, 0] - pc[:, 0]) ** 2 + (pb[:, 1] - pc[:, 1]) ** 2)
    c = np.sqrt((pc[:, 0] - pa[:, 0]) ** 2 + (pc[:, 1] - pa[:, 1]) ** 2)

    s = (a + b + c) * 0.5   # Semi-perimeter of triangle

    area = np.sqrt(         # Area of triangle by Heron's formula
        s * (s - a) * (s - b) * (s - c)
    )  

    filter = (              # Radius Filter based on alpha value
        a * b * c / (4.0 * area) < 1.0 / alpha
    )  
    
    edges = tri[filter]     # Filtering the edges

    edges = [
        tuple(sorted(combo)) for e in edges for combo in itertools.combinations(e, 2)
    ]

    count = Counter(edges)  # count occurrences of each edge
    
    edges = [e for e, c in count.items() if c == 1] # keep only edges that appear one time (concave hull edges)
    edges = [(coords[e[0]], coords[e[1]]) for e in edges] # these are the coordinates of the edges that comprise the concave hull

    ml = MultiLineString(edges)
    poly = polygonize(ml)
    hull = unary_union(list(poly))
    hull_vertices = hull.exterior.coords.xy

    return edges, hull_vertices

def point_inside_polygon(point, polygon, tol=1e-8):
    """
    Check if a given point is inside a polygon.

    Args:
    - point: a 2D point as a numpy array of shape (2,)
    - polygon: a numpy array of shape (N, 2) containing the vertices of the polygon
    - tol: tolerance for considering the point inside the polygon

    Returns:
    - True if the point is inside the polygon, False otherwise
    """

    n = len(polygon)
    inside = False

    # loop through all edges of the polygon
    for i in range(n):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % n]

        # check if point is on the edge
        if np.linalg.norm(np.cross(p2 - p1, point - p1)) < tol and np.dot(p2 - point, p1 - point) < tol:
            return False  # point is on the edge

        # check if point is to the left of the edge
        if p1[1] < p2[1]:
            if p1[1] < point[1] < p2[1] and np.cross(p2 - p1, point - p1) > tol:
                inside = not inside
        else:
            if p2[1] < point[1] < p1[1] and np.cross(p2 - p1, point - p1) < -tol:
                inside = not inside

    return inside

def get_nearest_front_cones(car_pos:np.ndarray,car_yaw:float, cone_positions:np.ndarray, count:int, max_distance=10.0, min_distance=0.1):
    """
    Returns the nearest cones in front of the car in ascending order of distance.
    """
    distances = cdist([car_pos], cone_positions)[0]                                                                                 # Calculate the distance between the car and each cone
    mask = np.array([np.linalg.norm(cone_pos - car_pos) > min_distance for cone_pos in cone_positions])                             # Ignore cones that are too close to the car
    mask = np.logical_and(mask, np.array([np.linalg.norm(cone_pos - car_pos) < max_distance for cone_pos in cone_positions]))       # Ignore cones that are too far away from the car
    mask = np.logical_and(mask, [np.dot(cone_pos - car_pos, np.array([np.cos(car_yaw), np.sin(car_yaw)])) > 0 for cone_pos in cone_positions])  # Ignore cones that are behind the car
    return cone_positions[mask][np.argsort(distances[mask])][:count]                                                                # Return the nearest cones in front of the car sorted by distance


def extract_waypoints(cone_positions:np.ndarray, car_pos:np.ndarray) -> np.ndarray:
    """
    Extracts waypoints from the given cone positions, this corresponds to the midpoints of the delaunay edges that are inside the boundary polygon.
    """
    boundary_polygon_points = concave_hull(cone_positions, 0.5)
    tri = Delaunay(cone_positions)
    midpoints = np.empty((0, 2), dtype=np.float64)
    for i in range(len(tri.simplices)):
        # Calculate the midpoint of each triangle
        m1 = (cone_positions[tri.simplices[i][0]] + cone_positions[tri.simplices[i][1]]) / 2
        m2 = (cone_positions[tri.simplices[i][0]] + cone_positions[tri.simplices[i][2]]) / 2
        m3 = (cone_positions[tri.simplices[i][1]] + cone_positions[tri.simplices[i][2]]) / 2
        # # Add the midpoints to the valid list of midpoints if they are not already in the list and inside the boundary polygon
        
        if (m1 not in midpoints) and point_is_inside_polygon(m1, boundary_polygon_points):
            midpoints = np.append(midpoints, [m1], axis=0)
        if (m2 not in midpoints) and point_is_inside_polygon(m2, boundary_polygon_points):
            midpoints = np.append(midpoints, [m2], axis=0)
        if (m3 not in midpoints) and point_is_inside_polygon(m3, boundary_polygon_points):
            midpoints = np.append(midpoints, [m3], axis=0)
        
    midpoints = nnb_sorting(midpoints, car_pos)
    return midpoints