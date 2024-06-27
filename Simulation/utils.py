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
