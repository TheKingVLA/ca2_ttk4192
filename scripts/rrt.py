#!/usr/bin/env python3

import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ca2_ttk4192.srv import isThroughObstacle, isThroughObstacleRequest, isInObstacle, isInObstacleRequest, positionControl, positionControlRequest

# 4.a
import math
import random

point_in_obstacle_service = rospy.ServiceProxy('point_in_obstacle', isInObstacle)

def isInObstacle(vex, radius):

    vex_pos = Point(vex[0], vex[1], 0.0)

    request = isInObstacleRequest(vex_pos, radius)
    response = point_in_obstacle_service(request)

    return response



path_through_obstacle_service = rospy.ServiceProxy('path_through_obstacle', isThroughObstacle)

def isThruObstacle(p0, p1, radius):

    p0_pos = Point(p0[0], p0[1], 0.0)
    p1_pos = Point(p1[0], p1[1], 0.0)

    request = isThroughObstacleRequest(p0_pos, p1_pos, radius)
    response = path_through_obstacle_service(request)

    return response

def get_marker(type, pos, size, color, identity):

    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = type
    marker.id = identity
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.action = Marker.ADD

    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = 0.001
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    return marker


def get_edge_as_marker(first_point, second_point, color, identity, thickness=0.025):

    edge_marker = get_marker(Marker.LINE_STRIP, [0,0], thickness, color, identity)
    
    p0_point = Point(first_point[0], first_point[1], 0.0)
    p1_point = Point(second_point[0], second_point[1], 0.0)
    edge_marker.points.append(p0_point)
    edge_marker.points.append(p1_point)
    
    return edge_marker

# 4.a <--
def get_distance(p0, p1):
    return math.hypot(p1[0] - p0[0], p1[1] - p0[1])

def get_random_point(xmin, xmax, ymin, ymax, goal, goal_bias):
    if random.random() < goal_bias:
        return goal
    return [random.uniform(xmin, xmax), random.uniform(ymin, ymax)]

def steer(nearest_point, random_point, step_size):
    dx = random_point[0] - nearest_point[0]
    dy = random_point[1] - nearest_point[1]
    distance = math.hypot(dx, dy)

    if distance <= step_size:
        return [random_point[0], random_point[1]]
    
    scale = step_size / distance
    return [nearest_point[0] + scale * dx, nearest_point[1] + scale * dy]

def reconstruct_path(parent, endpos):
    path = [list(endpos)]
    current = tuple(endpos)

    while parent[current] is not None:
        current = parent[current]
        path.append([current[0], current[1]])

    path.reverse()
    return path

def rrt(startpos, endpos, obstacle_radius, tree_marker, marker_identity, edge_color):
    x_min = -1.5
    x_max = 5.8
    y_min = -1.5
    y_max = 5.8
    step_size = 0.35
    goal_bias = 0.10
    goal_tolerance = 0.40
    max_iterations = 5000

    nodes = [list(startpos)]
    parent = {tuple(startpos): None}
    final_path = None

    for i in range(max_iterations):
        random_point = get_random_point(x_min, x_max, y_min, y_max, list(endpos), goal_bias)

        nearest_point = min(nodes, key=lambda node: get_distance(node, random_point))
        new_point = steer(nearest_point, random_point, step_size)

        if isInObstacle(new_point, obstacle_radius).inObstacle:
            continue

        if isThruObstacle(nearest_point, new_point, obstacle_radius).throughObstacle:
            continue

        if tuple(new_point) in parent:
            continue

        nodes.append(new_point)
        parent[tuple(new_point)] = tuple(nearest_point)

        edge_marker = get_edge_as_marker(nearest_point, new_point, edge_color, marker_identity)
        marker_identity += 1
        tree_marker.markers.append(edge_marker)

        if get_distance(new_point, endpos) <= goal_tolerance:
            if not isThruObstacle(new_point, endpos, obstacle_radius).throughObstacle:
                parent[tuple(endpos)] = tuple(new_point)

                edge_marker = get_edge_as_marker(new_point, endpos, edge_color, marker_identity)
                marker_identity += 1
                tree_marker.markers.append(edge_marker)

                final_path = reconstruct_path(parent, endpos)
                break

    if final_path is not None:
        path_edge_color = [0/256, 114/256, 178/256]
        
        for i in range(len(final_path) - 1):
            first_point = final_path[i]
            second_point = final_path[i + 1]
            edge_marker = get_edge_as_marker(first_point, second_point, path_edge_color, marker_identity, thickness=0.05)
            marker_identity += 1
            tree_marker.markers.append(edge_marker)
    else:
        rospy.logwarn("RRT did not find a path.")

    return final_path, marker_identity, tree_marker
# -->

if __name__ == '__main__':



    # -----------------
    # Init the RRT node
    rospy.init_node('RRT')

    # 4.a
    rospy.wait_for_service("point_in_obstacle")
    rospy.wait_for_service("path_through_obstacle")
    rospy.wait_for_service("/position_control")

    # -----------------------------------------------
    # The start and end positions of the "short maze"
    startpos = (0.0, 0.0)
    endpos = (4.5, 5.0)

    # 4.a
    short_maze_startpos = startpos
    short_maze_endpos = endpos

    # -------------------------------------------------------------------------
    # The start and end positions of the bonus task with the "complicated maze"
    startpos = (0.0, 0.0)
    endpos = (4.5, 9.0)


    
    # ---------------------------------------------------------------
    # Example of how you can check if a point is inside an obstacle: 

    obstacle_radius = 0.3
    vex_pos = [0.0, 0.0]
    response = isInObstacle(vex_pos, obstacle_radius)
    print("point_in_obstacle_service response: ")
    print(response)


    
    # -----------------------------------------------------------------------
    # Example of how you can check if the straightline path between two points 
    # goes through an obstacle:

    obstacle_radius = 0.3
    first_point = [0.0, 0.0]
    second_point = [1.0, 1.0]
    response = isThruObstacle(first_point, second_point, obstacle_radius)
    print("path_through_obstacle_service response: ")
    print(response)



    # -----------------------------------------------------------------------
    # Example of how you can visualize a graph using a MarkerArray publisher:

    list_of_positions = [[0.0, 0.0], [0.0, 2.0], [1.0, 2.0], [1.0, 1.0]]

    tree_publisher = rospy.Publisher('tree_marker', MarkerArray, queue_size=10)
    tree_marker = MarkerArray()
    marker_identity = 0

    # Create a blue-green square representing the start
    start_rgb_color = [0/256, 158/256, 115/256]
    start_marker_size = 0.2
    start_marker = get_marker(Marker.CUBE, short_maze_startpos, start_marker_size, start_rgb_color, marker_identity)
    marker_identity += 1

    # Create a vermillion square representing the goal
    end_rgb_color = [213/256, 94/256, 0/256]
    end_marker_size = 0.2
    end_marker = get_marker(Marker.CUBE, short_maze_endpos, end_marker_size, end_rgb_color, marker_identity)
    marker_identity +=1

    tree_marker.markers.append(start_marker)
    tree_marker.markers.append(end_marker)

    
    # Create reddish purple edges 
    edge_color = [204/256, 121/256, 167/256]
    
    # 4.a <--
    startpos = short_maze_startpos
    endpos = short_maze_endpos

    final_path, marker_identity, tree_marker = rrt(
        startpos,
        endpos,
        obstacle_radius,
        tree_marker,
        marker_identity,
        edge_color
    )
    # -->

    rospy.Rate(0.5).sleep() # needs to a bit for the publisher to start, a bit weird. 
    tree_publisher.publish(tree_marker)



    # ---------------------------------------------------------------------------
    # Example of how you can make the turtlebot go through a sequence of positions
    # using the position controller.

    # Note that it will just move in a straight line between the current position and the
    # desired position, so it will crash into possible obstacles. 
    # It is also not very well tuned (and quite slow).

    # 4.a
    if final_path is not None:
        list_of_positions = final_path[1:]
    else:
        list_of_positions = []    

    position_control = rospy.ServiceProxy('/position_control', positionControl)

    for position in list_of_positions:
        rospy.wait_for_service('/position_control')
        request = Point(position[0], position[1], 0.0)
        response = position_control(request)
        print(response)