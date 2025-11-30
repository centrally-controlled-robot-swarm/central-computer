from math import cos, sin, radians, degrees, atan2, sqrt
import numpy as np
import matplotlib.pyplot as plt

def heading_error(curr_pos, goal_pos, heading):
    curr_pos_x, curr_pos_y = curr_pos
    goal_pos_x, goal_pos_y = goal_pos
    
    hypotenuse = 1
    interpolated_pos_x = curr_pos_x + hypotenuse * cos(radians(heading))
    interpolated_pos_y = curr_pos_y + hypotenuse * sin(radians(heading))
    
    curr_vector = np.array([
        interpolated_pos_x - curr_pos_x,
        interpolated_pos_y - curr_pos_y
    ])

    goal_vector = np.array([
        goal_pos_x - curr_pos_x,
        goal_pos_y - curr_pos_y
    ])

    dot = np.dot(curr_vector, goal_vector)
    det = np.cross(curr_vector, goal_vector)

    angle = degrees(atan2(det, dot))

    plot(curr_pos, goal_pos, heading)
    return angle


def distance_error(curr_pos, goal_pos):
    curr_pos_x, curr_pos_y = curr_pos
    goal_pos_x, goal_pos_y = goal_pos

    x_distance = goal_pos_x - curr_pos_x
    y_distance = goal_pos_y  - curr_pos_y

    return sqrt(x_distance**2 + y_distance**2)


def plot(curr_pos, goal_pos, heading, show=True):
    """
    Plot heading vector (red) and goal vector (blue) from curr_pos.
    heading in degrees (0 = +x, ccw positive).
    """
    cx, cy = curr_pos
    gx, gy = goal_pos
    rad = radians(heading)
    heading_vec = np.array([cos(rad), sin(rad)])
    goal_vec = np.array([gx - cx, gy - cy])

    plt.figure()
    # heading vector (unit)
    plt.quiver(cx, cy, heading_vec[0], heading_vec[1], angles='xy', scale_units='xy', scale=1, color='r', label='heading')
    # goal vector (scaled to its length)
    plt.quiver(cx, cy, goal_vec[0], goal_vec[1], angles='xy', scale_units='xy', scale=1, color='b', label='goal')
    plt.scatter([cx, gx], [cy, gy], color=['k','b'])
    plt.text(cx, cy, ' curr', va='bottom', ha='left')
    plt.text(gx, gy, ' goal', va='bottom', ha='left')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    if show:
        plt.show()


if __name__=="__main__":
    curr_pos = (-5, 5)
    goal_pos = (-6, -6.25)
    print(heading_error(curr_pos, goal_pos, 0))
    print(distance_error(curr_pos, goal_pos))