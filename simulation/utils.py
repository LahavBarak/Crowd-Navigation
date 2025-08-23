import math
import random
import numpy as np
from numpy import cos, sin

def distance_point_to_segment(px, py, ax, ay, bx, by):
    # Vectorized approach
    p = np.array([px, py])
    a = np.array([ax, ay])
    b = np.array([bx, by])
    ab = b - a
    ap = p - a
    ab_len_sq = np.dot(ab, ab)
    t = np.dot(ap, ab) / ab_len_sq

    t = np.clip(t, 0, 1)
    nearest = a + t * ab
    dist = np.linalg.norm(nearest - p)
    return dist

def minimal_distance(circle_center, circle_radius, rectangle_vertices):
    cx, cy = circle_center
    r = circle_radius
    vertices = np.array(rectangle_vertices)

    # Distances from circle center to rectangle vertices
    circle_center_arr = np.array([cx, cy])
    distances_to_vertices = np.linalg.norm(vertices - circle_center_arr, axis=1) - r
    min_dist = np.min(distances_to_vertices)

    # Distances from circle center to rectangle edges
    for i in range(len(vertices)):
        ax, ay = vertices[i]
        bx, by = vertices[(i + 1) % len(vertices)]
        edge_distance = distance_point_to_segment(cx, cy, ax, ay, bx, by) - r
        min_dist = min(min_dist, edge_distance)

    return min_dist

def get_vertices(x_cm, y_cm, theta, length, width):
        '''
        given x,y and theta - return vertex locations of the rectangle.

        reset vertices to parallel position, displaced to new center,
        then rotate by theta radians to reflect the theta of the robot
        '''
        vertices = ([x_cm-(length/2),y_cm-(width/2)],
                    [x_cm-(length/2),y_cm+(width/2)],
                    [x_cm+(length/2),y_cm+(width/2)],
                    [x_cm+(length/2),y_cm-(width/2)])
        cm = np.array([x_cm,y_cm])
        rotated_vertices = []
        for vertex in vertices:
            np_vertex = np.array(vertex)
            rotated_vertex = np.dot(np_vertex-cm,R(theta))
            rotated_vertex += cm
            rotated_vertices.append((rotated_vertex[0],
                                    rotated_vertex[1]))
        return rotated_vertices

def R(theta):
        return np.array([[cos(theta),-sin(theta)],
                        [sin(theta),cos(theta)]])

def line_segments_intersect(p1, p2, p3, p4):
        """
        Check if two line segments (p1 to p2) and (p3 to p4) intersect.
        Uses the parametric equation method.
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:
            return False  # Parallel lines, no intersection
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denom
        
        return 0 <= t <= 1 and 0 <= u <= 1

def get_max_heading_change(wheelbase, v_max, delta_max, duration_max):
    """
    Compute φ_max = (v_max * |tan(delta_max)| / wheelbase) * duration_max,
    i.e. the maximum possible heading change (in radians) if you hold
    |Δ| = delta_max, v = v_max, for time = duration_max.
    """
    return (v_max * abs(math.tan(delta_max)) / wheelbase) * duration_max

def sample_xy(x_init, y_init, theta_init,
                                search_radius, half_cone_angle):
    """
    Uniformly sample a point (x,y) inside:
      1) the circle of radius = search_radius centered at (x_init, y_init),
      2) AND inside the forward-facing cone of half-angle = half_cone_angle
         about direction theta_init.
    
    We do this by picking:
      r ∈ [0..search_radius], with PDF ∝ r  (⇒ r = search_radius * √u),
      α ∈ [theta_init - half_cone_angle .. theta_init + half_cone_angle],
    then convert (r, α) → (x, y) in Cartesian.
    """
    # 1) pick a uniform‐area radius
    u = random.random()           # ∈ [0,1)
    r = search_radius * math.sqrt(u)

    # 2) pick a uniform angle in [θ_init - half_cone_angle .. θ_init + half_cone_angle]
    α = theta_init + random.uniform(-half_cone_angle, half_cone_angle)

    # 3) convert back to Cartesian
    x = x_init + r * math.cos(α)
    y = y_init + r * math.sin(α)
    return x, y