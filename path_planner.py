"""
modified from:
https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2


resources:
https://www.youtube.com/watch?v=pKnV6ViDpAI
https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
https://www.redblobgames.com/pathfinding/a-star/introduction.html

"""
"""
INPUT:
- rover pos

STATUS:
- reformatting to 3 way points instead of start_node, end_node

TODO:

- Compare location to expected location

- create metrics for movement
        - What I thnk is happening based on loction history and when to intervene

- Set go/no go areas-- CHANGE THE MAP-MAKER

- plan most efficient path through given 3 points and set target order

- Think about possible limitting physical cases

"""



from collections import deque
import math as m
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import random
from map_maker import MapMaker
import math
import serial

"""
Node functions:
TODO: documentation
"""

class Node():
    def __init__(self, pos=None, parent=None, g=None, h=None, size_map=[7.75, 16]):
        self.pos = pos
        self.parent = parent
        self.g = g
        self.h = h

    def get_f(self):
        return self.g + self.h

    def print_attributes(self):
        f = self.get_f()
        print("POS: {}, PARENT: {}, G: {}, H: {}, F: {}".format(self.pos, self.parent, self.g, self.h, f))

    def __eq__(self, other):
        return self.pos == other.pos

"""
PathPlanner functions:
- set_map
    - sets global variable 'self.map'
- set_start_node
    - sets global variable 'self.start_node'
- set_end_node
    - sets global variable 'self.end_node'
- calculate_h
    - returns cost from current node to goal
- calculate_g
    - returns g cost from current to neighbor
- get_neighbors
    - returns list of neighboring nodes
- plan_path
    - Plans a path from self.start_node to self.end_node using A*
- initialize_plot
    - starts heat map of map with start and end nodes overlaid. Path is drawn over this in other functions. Called by plan_path
- update_plot
    - Overlays scatter plot with current node and neighbors over initialized plot. Called by plan_path
- plot_final_path
    - Plots the map with path overlain. Shows start and end nodes and the path in between.
- run

More:
- pos_from_coordinates
- coordinates_from_pos
- update_status


Notes:
- Origin of map is in the bottom left corner. X goes to the right, Y goes up. However, pos goes [row, column] from top left corner. 

"""

class PathPlanner():
    def __init__(self, start_coord=[0,0]):
        self.map = []
        self.start_node = None
        self.end_node = None
        self.path = None
        self.real_map_size = [7.75, 15] # size in feet of map.
        self.completed_path = []
        self.target_nodes = []
        self.start_coord = start_coord
        self.accuracy_check = 0 # add 1 when location not what expected

        # self.ser = serial.Serial('/dev/ttyUSB0', 9600)
        # self.ser.close()
        # self.ser.open() 

    def set_map(self, map):
        '''
        Function: set_map
        Inputs: map (ndarray)
        Default: None
        Returns: None
        Calls: None
        Notes: Sets global variable 'self.map'
        '''
        print("PathPlanner: RECEIVED MAP")
        self.map = map
        return

    def set_targets(self, coordinates=[[0,0], [3,3], [7.75, 15]]):
        '''
        Function: set_targets
        Inputs: Real life coordinates of targets -- list of [x,y]
        Default: list of coordinates - start [0,0], end [7.75, 15] real map coord frame
        Returns: None
        Calls: Node, pos_from_coordinates, self.target_nodes
        Notes: makes list of target nodes (payload drop points), unsorted
        '''
        for i in range(0, len(coordinates)):
            target_coord = coordinates[i]
            target_pos = self.pos_from_coordinates(target_coord)
            print("PathPlanner: SETTING TARGET NODE {}: Coordinate: {}  Position: {}".format(i, target_coord, target_pos))
            target_node = Node(pos=target_pos)
            self.target_nodes.append(target_node)

        return

    def calculate_h(self, pos):
        '''
        Function: calculate_h
        Inputs: pos (tuple)
        Default: None
        Returns: h (distance from current node to end node)
        Calls:
        Notes: Returns cost from current node to goal
        '''
        # calculate distance from current node to end node
        x = pos[1] # column index
        y = pos[0] # row index

        end_x = self.end_node.pos[1]
        end_y = self.end_node.pos[0]

        h = m.sqrt((x-end_x)**2 + (y-end_y)**2) * 500 # simple triangle dist formula to end: a^2 + b^2 = c^2
        return h

    def calculate_g(self, positions):
        '''
        Function: calculate_g
        Inputs: positions (list of position tuples)
        Default: None
        Returns: total_g (list of g cost for each pos given)
        Calls: self.map
        Notes: Returns cost to node from previous node. Cost currently calculated based on slope
        TODO: modify to correct cost calculations
        '''
        # Generic g cost from current to neighboring points
        scale_factor = 1
        generic_cost = [1.4, 1, 1.4,
                1, 1,
                1.4, 1, 1.4]
        generic_cost = [x*scale_factor for x in generic_cost]

        # add weight of value (slope) in array
        slope_weight = []
        ##print(positions)

        for i in range(0,len(positions)):
            current_pos = positions[i]
            if current_pos: # If the position exists
                slope = abs(self.map[current_pos[0],current_pos[1]])
                slope_weight.append(slope)
            else:
                slope_weight.append(0)

        total_g = np.add(generic_cost,slope_weight)
        ##print(total_g)
        return total_g

    def get_neighbors(self, node):
        '''
        Function: get_neighbors
        Inputs: node (Node)
        Default: None
        Returns: neighbors (list of Nodes)
        Calls: self.map
        Notes: Returns positions of currect node's neighbors in 1d array from top left to bottom right. Deals with corner, edge, and general cases.
        '''
        """
        Return positions of current node's neighbors in 1d array from top left to bottom right

        This deals with corner and edge cases, and calls all else the general case
        """
        
        neighbors = []
        row = node.pos[0]
        col = node.pos[1]

        # get max dimensions of map for corner cases
        max_row, max_column = self.map.shape

        # EDGE CASES
        # top row
        if (node.pos[0] == 0): 
            if (node.pos[1] == 0):
                # top left corner
                #print("NEIGHBORS: TOP LEFT")
                neighbors = [None, None, None,
                            None, [row, col+1],
                            None, [row+1, col], [row+1, col+1]]
            elif (node.pos[1] == max_column-1):
                # top right corner
                #print("NEIGHBORS: TOP RIGHT")
                neighbors = [None, None, None,
                        [row, col-1], None,
                        [row+1, col-1], [row+1, col], None]
            else:
                # just top row
                #print("NEIGHBORS: TOP SIDE")
                neighbors = [None, None, None,
                        [row, col-1], [row, col+1],
                        [row+1, col-1], [row+1, col], [row+1, col+1]]

        # bottom row
        elif (node.pos[0] == max_row-1):
            if (node.pos[1] == 0): 
                # bottom left corner
                #print("NEIGHBORS: BOTTOM LEFT")
                neighbors = [None, [row-1, col], [row-1, col+1],
                            None, [row, col+1],
                            None, None, None]
            elif (node.pos == [max_row-1,max_column-1]): 
                # bottom right corner
                #print("NEIGHBORS: BOTTOM RIGHT")
                neighbors = [[row-1, col-1], [row-1, col], None,
                            [row, col-1], None,
                            None, None, None]
            else:
                # just bottom row
                #print("NEIGHBORS: BOTTOM SIDE")
                neighbors= [[row-1, col-1], [row-1, col], [row-1, col+1],
                            [row, col-1], [row, col+1],
                            None, None, None]

        # left column and not corners
        elif (node.pos[1] == 0 and node.pos[0] != 0 and node.pos[0] != max_row-1):
            #print("NEIGHBORS: LEFT SIDE")
            neighbors = [None, [row-1, col], [row-1, col+1],
                            None, [row, col+1],
                            None, [row+1, col], [row+1, col+1]]
        
        # right column and not corners
        elif (node.pos[1] == max_column-1 and node.pos[0] != 0 and node.pos[0] != max_row-1):
            #print("NEIGHBORS: RIGHT SIDE")
            neighbors = [[row-1, col-1], [row-1, col], None,
                        [row, col-1], None,
                        [row+1, col-1], [row+1, col], None]

        # general case
        else:
            #print("NEIGHBORS: GENERAL CASE")
            neighbors = [[row-1, col-1], [row-1, col], [row-1, col+1],
                        [row, col-1], [row, col+1],
                        [row+1, col-1], [row+1, col], [row+1, col+1]]

        return neighbors

    def plan_path(self, plot_path=False):
        '''
        Function: plan_path
        Inputs: None
        Default: plot_path = False (won't plot path in progress)
        Returns: path (list of Nodes from start to end)
        Calls: self.initialize_plot(), self.get_neighbors(), self.calculate_g(), self.calculate_h(), self.update_plot()
        Notes: Plans a path from give start_node to given end_node using A*
        '''

        print("PathPlanner: BEGIN PATH PLANNING")
        if plot_path:
            print("PathPlanner: PLOTTING PATH")
            self.initialize_plot()

        # INITIALIZE NODE SETS
        open_set = [] # contains unvisited nodes in cost order by f (lowest to the left)
        created_set = [] # contains all created nodes

        # INITIALIZE PATH WITH START NODE AND ITS NEIGHBORS
        current_node = self.start_node
        #print(current_node.pos)
        created_set.append(current_node)
        neighbors_pos = self.get_neighbors(current_node)
        g_costs = self.calculate_g(neighbors_pos)

        for i in range(0, len(neighbors_pos)):
            if neighbors_pos[i]:
                # create neighbor node
                parent = current_node
                pos = neighbors_pos[i]
                g = g_costs[i]
                h = self.calculate_h(pos)
                new_node = Node(pos=pos, parent=parent, g=g, h=h)

                # add neighbor to open_set
                open_set.append(new_node)
                created_set.append(new_node)

        open_set = sorted(open_set, key=Node.get_f)
        
        # PLAN PATH
        steps = 0
        while (len(open_set) > 0):
            if plot_path:
                self.update_plot(current_pos = current_node.pos, neighbors_pos = neighbors_pos)

            current_node = open_set[0]
            # remove current node from open set
            open_set.pop(0)

            if (current_node == self.end_node):
                print("END NODE REACHED")
                break

            neighbors_pos = self.get_neighbors(current_node)
            #print(current_node.pos)
            g_costs = self.calculate_g(neighbors_pos)

            for i in range(0, len(neighbors_pos)):
                if neighbors_pos[i]:
                    # create node with pos only for comparison
                    pos = neighbors_pos[i]
                    new_node = Node(pos=pos)

                    # check if already created
                    if any(node == new_node for node in created_set):
                        ##print("NODE ALREADY EXISTS")
                        # check if in open set
                        if any(node == new_node for node in open_set):
                            ##print("NODE IS IN OPEN SET")
                            # get old node
                            index = open_set.index(new_node)
                            #old_node = open_set[index]
                            ##print("Node before change:")
                            #open_set[index].#print_attributes()

                            # compare past and current g cost
                            old_g = open_set[index].g
                            new_g = current_node.g + g_costs[i]

                            # update the parent if the new g cost is lower. Otherwise, leave alone
                            if (old_g > new_g):
                                ##print("UPDATING NODE")
                                open_set[index].parent = current_node
                                open_set[index].g = new_g

                            ##print("Node after change:")
                            #open_set[index].#print_attributes()
                    
                    # if it isn't created, create it
                    else:
                        new_node.parent = current_node
                        new_node.g = g_costs[i]
                        new_node.h = self.calculate_h(pos)
                        open_set.append(new_node)
                        created_set.append(new_node)
            steps = steps+1
            # sort open set again
            open_set = sorted(open_set, key=Node.get_f)

        print("PathPlanner: END PATH PLANNING")
        plt.show()

        # curious to see how many steps it took
        ##print("STEPS TO END: {}".format(steps))
        path = []
        while (current_node != self.start_node):
            path.append(current_node.pos)
            current_node = current_node.parent
        path.append(current_node.pos) # add start node
        
        # reverse so it's from start to end
        path = path[::-1]
        self.path = path
        return

    def initialize_plot(self):
        '''
        Function: initialize_plot
        Inputs: None
        Default: None
        Returns: None
        Calls: self.map
        Notes: Makes heat map of map with start and end nodes overlaid. The path is later drawn over this.
        start_x and start_y correspond to col, row indeces, as do end_x and end_y
        '''
        print("INITIALIZING PLOT")
        sns.heatmap(self.map, cmap="YlGnBu")

        thickness = 100

        #plot start node (bright green)
        start_x = self.start_node.pos[1] + .5
        start_y = self.start_node.pos[0] + .25
        start = plt.scatter(start_x, start_y, marker='o', s=thickness, color=[0,1,0])

        # plot end node (bright blue)
        end_x = self.end_node.pos[1] + .5
        end_y = self.end_node.pos[0] + .25
        end = plt.scatter(end_x, end_y, marker='o', s=thickness, color=[0,1,1])

        # will need to comment this out if you choose to plot neighbors
        plt.legend((start, end), ('Start', 'End'))
        return

    def update_plot(self, current_pos, neighbors_pos):
        """
        '''
        Function: update_plot
        Inputs: current_pos, neighbors_pos
        Default: None
        Returns: None
        Calls: None
        Notes: Overlays scatter plot with current node and neighbors over initialized plot
        x and y correspond to col, row indeces
        '''
        """
        # plot neighbors 
        x = []
        y = []
        for neighbor in neighbors_pos:
            if neighbor:
                x.append(neighbor[1]+.15)
                y.append(neighbor[0]+.15)

        neighbors = plt.scatter(x, y, marker = 'o', s = 10, color = [0, 0, 1])
        current = plt.scatter(current_pos[1], current_pos[0], marker = 'o', s = 5, color = [1, .1, .5])

        plt.legend((neighbors, current), ('Neighbors', 'Current'))

        plt.pause(0.05)
        return

    def plot_final_path(self):
        '''
        Function: plot_final_path
        Inputs: path (list of nodes)
        Default: None
        Returns: None
        Calls: self.map, self.path
        Notes: Plots the map with path overlain. Shows start and end nodes and the path in between.
        start_x and start_y correspond to col, row indeces, as do end_x and end_y
        '''
        """
        This plots the heatmap with the path overlain. It shows the start and end node and the path in between.
        """
        print("PLOTTING FINAL PATH")        
        x = []
        y = []
        for pos in self.path:
            x.append(pos[1]+.5) # offset to look good on heat map
            y.append(pos[0]+.5)

        # plot heat map
        sns.heatmap(self.map, cmap="YlGnBu")

        thickness = 100

        #plot start node (bright green)
        start_x = self.start_node.pos[1] + .5
        start_y = self.start_node.pos[0] + .25
        start = plt.scatter(start_x, start_y, marker='o', s=thickness, color=[0,1,0])

        # plot end node (bright blue)
        end_x = self.end_node.pos[1] + .5
        end_y = self.end_node.pos[0] + .25
        end = plt.scatter(end_x, end_y, marker='o', s=thickness, color=[0,1,1])

        # plot path
        path_plot = plt.scatter(x, y, marker='o', s=thickness, color=[1, .5, 0])

        plt.legend((start, end, path_plot), ('Start', 'End', 'Path'))

        plt.show()
        return

    def coordinates_from_pos(self, pos):
        '''
        Function: coordinates_from_pos
        Inputs: pos (row, column in map array)
        Default: None
        Returns: real_x, real_x (real world coordinates with bottom left origin in map)
        Calls: self.real_map, self.map
        Notes: Used in [TODO: fill]
        '''
        # real map size in [x,y] (bottom left origin)
        real_map_x = self.real_map_size[0]
        real_map_y = self.real_map_size[1]

        # map array size in [rows, columns] - corresponds to y,x. Translated to same coordinate frame:
        map_array_x = self.map.shape[1]
        map_array_y = self.map.shape[0]

        # divide feet by array size
        x_multiplier = real_map_x / map_array_x
        y_multiplier = real_map_y / map_array_y

        # translate array indeces to real life coordinates
        real_x = pos[1] * x_multiplier
        real_y = (map_array_y - pos[0]) * y_multiplier

        # make sure coordinates exist:
        if real_x < 0:
            print("WARNING: PathPlanner: Coordinate X ({}) < 0, setting to 0".format(real_x))
            real_x = 0
        if real_x > real_map_x:
            print("WARNING: PathPlanner: Coordinate X ({}) > map width, setting to map width ({})".format(real_x, real_map_x))
            real_x = real_map_x
        if real_y < 0:
            print("WARNING: PathPlanner: Coordinate Y ({}) < 0, setting to 0".format(real_y))
            real_y = 0
        if real_y > real_map_y:
            print("WARNING: PathPlanner: Coordinate Y ({}) > map height, setting to map height ({})".format(real_map_y))
            real_y = real_map_y

        return [real_x, real_y]

    def pos_from_coordinates(self, coordinate):
        '''
        Function: pos_from_coordinates
        Inputs: coordinates (real_x, real_y in real world coordinate frame)
        Default: None
        Returns: row, col (indeces in map array)
        Calls: self.real_map, self.map
        Notes: Used in [TODO]
        '''
        # real map size in [x,y] (bottom left origin)
        real_map_x = self.real_map_size[0]
        real_map_y = self.real_map_size[1]

        # map array size in [rows, columns] - corresponds to y,x. Translated to same coordinate frame:
        map_array_x = self.map.shape[1]
        map_array_y = self.map.shape[0]

        # divide array size by feet
        row_multiplier = map_array_y / real_map_y
        col_multiplier = map_array_x / real_map_x

        # translate coordinates to array indeces
        row = int((real_map_y - coordinate[1]) * row_multiplier)
        col = int(coordinate[0] * col_multiplier)

        # make sure coordinates exist:
        if row < 0:
            print("WARNING: PathPlanner: Pos Row ({}) < 0, setting to 0".format(row))
            row = 0
        if col >= map_array_x:
            print("WARNING: PathPlanner: Pos Col ({}) > num cols, setting to max col ({})".format(col, map_array_x-1))
            col = map_array_x - 1
        if col < 0:
            print("WARNING: PathPlanner: Pos Col ({}) < 0, setting to 0".format(col))
            col = 0
        if row >= map_array_y:
            print("WARNING: PathPlanner: Pos Row ({}) > num rows, setting to max row ({})".format(row, map_array_y-1))
            row = map_array_y - 1

        return [row, col]

    def get_action(self, coordinate=None, angle=None, message=None, test=False):
        '''
        Function: get_action
        Inputs: coordinate (x, y in map), angle (counterclockwise from x, radians)
        Default: None
        Returns: message (string to rover containing delta_angle and delta_distance)
        Calls: self.path, self.coordinates_from_pos, self.angle_between_coordinates, self.calculate_angular_movement, self.calculate_linear_movement, self.make_message
        Notes: Follows along most recently planned path
        - Checks if path exists (if not, target reached or error)
        - Checks if incoming message is stuck (sets pos to no-go area, backs up, re-plans)
        TODO: Format of stuck/unstuck message
        '''
        # get message/position update from rover (through main)
        # TODO: need to talk to carl about potential messages
        current_coord = coordinate # x, y in map
        current_angle = angle # counterclockwise from x, radians
        incoming_message = message # stuck/other status

        if incoming_message == "stuck":
            # TODO: re-path plan
            current_pos = self.pos_from_coordinates(current_coord)
            self.go_nogo(current_pos)
            # Carl does backup maneuver (goes to previous location)?
            # I do backup maneuver
            return
        
        # Completed path starts at start_node
        self.completed_path.append(self.path[0])
        self.path.pop(0)

        if not self.path:
            if current_coord == self.coordinates_from_pos(self.end_node.pos):
                print("Target reached!")
            else:
                print("WARNING: PathPlanner: NO PATH! Plan again to reach target")
                # TODO: re-plan path
            if test:
                return "ACTION: NO PATH", None, None
            return

        # Check accuracy (if current pos is where we expected to be)
        curr_pos = self.pos_from_coordinates(current_coord)
        expected_pos = self.completed_path[-1]
        tolerance = .5
        low = [x - tolerance for x in expected_pos]
        high = [x + tolerance for x in expected_pos]
        # if self.completed_path[-1] != self.pos_from_coordinates(current_coord):
        if (curr_pos > high) or (curr_pos < low):
            self.accuracy_check = self.accuracy_check + 1
            if self.accuracy_check > 3: # off three times in a row
                print("STOP! Rover is off course")
                if test:
                    return "ACTION: STOP", None, None
        else:
            self.accuracy_check = 0

        next_pos = self.path[0]
        target_pos = next_pos # row, col
        target_coord = self.coordinates_from_pos(target_pos)
        target_angle = self.angle_between_coordinates(current_coord, target_coord)
        
        # get needed change in angle:
        delta_angle = self.calculate_angular_movement(current_angle, target_angle)
        # get change in distance:
        delta_distance = self.calculate_linear_movement(current_coord, target_coord)

        if test:
            message = "DIST: %0.3f  ANGLE: %0.3f" % (delta_distance, delta_angle)
            return message, target_coord, target_angle

        else:
            message = self.make_message(delta_angle, delta_distance)    
            return message

    def make_message(self, angle=0, distance=0, size=6):
        '''
        Function: 
        Inputs:
        Default:
        Returns:
        Calls:
        Notes:
        '''
        # TODO: Check this format
        length = int(size/2)
        angle_str = str(angle)[:length]
        dist_str = str(distance)[:length]
        #message = angle_str + dist_str
        message = bytearray([angle, distance])
        # ^ this probably needs to change based on the format of angle and distance

        # print(message)
        # self.ser.write(message)
        return message
    
    def angle_between_coordinates(self, start_coordinate, end_coordinate):
        '''
        Function: 
        Inputs:
        Default:
        Returns:
        Calls:
        Notes:
        '''

        # gives you the angle from the start coordinate to end coordinate counterclockwise from x in the map coordinate system, in radians

        angle = 0

        if (start_coordinate == end_coordinate):
            return 0

        x1, y1 = start_coordinate[0], start_coordinate[1]
        x2, y2 = end_coordinate[0], end_coordinate[1]

        delta_x = x2 - x1
        delta_y = y2 - y1

        if (delta_y == 0):
            if delta_x > 0:
                # target directly to right
                angle = 0
            if delta_x < 0:
                # target directly to left
                angle = math.pi
        elif (delta_x == 0):
            if delta_y > 0:
                # target above
                angle = math.pi / 2
            if delta_y < 0:
                # target below
                angle = math.pi * (3/2)
        else:
            angle = np.arctan(delta_y/delta_x)
            # adjust angle to coordinate frame
            if (angle > 0):
                # either top right (no adjustments) or bottom left (add pi)
                if (x2 < x1):
                    angle = angle + math.pi
            if angle < 0:
                # either top left (subtract from pi) or bottom right (subtract from 2pi)
                if (x2 < x1): # top left
                    angle = math.pi - angle
                else: # bottom right
                    angle = (math.pi * 2) - angle
        
        return angle

    def calculate_angular_movement(self, current_angle, target_angle):
        '''
        Function: 
        Inputs:
        Default:
        Returns:
        Calls:
        Notes:
        '''
        # check which way to turn based on whether it's more or less than pi radians away
        # positive if counterclockwise, negative if clockwise

        delta_angle = 0

        if (current_angle == target_angle):
            delta_angle = 0

        elif (current_angle > target_angle):
            threshold_angle = target_angle + math.pi
            if (current_angle <= threshold_angle):
                # rotate clockwise (negative)
                delta_angle = -1*(current_angle - target_angle)
            else:
                # rotate counterclockwise (positive)
                delta_angle = 2*math.pi - (current_angle - target_angle)

        else:
            threshold_angle = current_angle + math.pi
            if (target_angle < threshold_angle):
                # rotate counterclockwise (positive)
                delta_angle = target_angle - current_angle
            else:
                # rotate clockwise (negative)
                delta_angle = -1*(2*math.pi - (target_angle - current_angle))

        return delta_angle

    def calculate_linear_movement(self, start_coordinate, end_coordinate):
        '''
        Function: 
        Inputs:
        Default:
        Returns:
        Calls:
        Notes:
        '''

        x1, y1 = start_coordinate[0], start_coordinate[1]
        x2, y2 = end_coordinate[0], end_coordinate[1]

        delta_x = x2 - x1
        delta_y = y2 - y1

        distance = np.sqrt(delta_x**2 + delta_y**2)

        return distance

    def go_nogo(self, pos, threshold_val=10000):
        '''
        Function: 
        Inputs:
        Default:
        Returns:
        Calls:
        Notes: want to update nogo if area not passable ("got stuck")
        '''
        self.map[pos[0], pos[1]] = threshold_val

    def run(self):
        '''
        Function: 
        Inputs:
        Default:
        Returns:
        Calls:
        Notes:
        '''

        print("BEGIN")
        print("END")

    def test_action(self):
        self.start_node = self.target_nodes[1]
        self.end_node = self.target_nodes[-1]
        self.plan_path(plot_path=False)
        
        current_coord = self.coordinates_from_pos(self.start_node.pos)
        current_angle = 0

        while self.path: # while there's a path to follow
            # print("Steps left in path: {}".format(len(self.path)))
            message, current_coord, current_angle = self.get_action(current_coord, current_angle, test=True)
            c0, c1 = current_coord[0], current_coord[1]
            current_coord = [c0-3, c1]
            print(message + " STEPS LEFT:{}".format(len(self.path)))
            # self.make_message(angle =.14159262, distance=164.345)

    def debug(self):
        self.start_node = self.target_nodes[1]
        self.end_node = self.target_nodes[-1]
        self.plan_path(plot_path=True)
        self.plot_final_path()
        print(self.path)

    def test_rover_movement(self):
        width_path = [[0,0], [1,0],[2,0],[3,0], [4,0], [5,0]]
        for i in range(0,len(width_path)):
            width_path[i] = self.pos_from_coordinates(width_path[i])
        self.path = width_path
        self.start_node = Node(pos = self.path[0])
        self.end_node = Node(pos = self.path[-1])
        # self.plot_final_path()

        # for i in range(0,len(self.path)):
        #     pos = self.path[i]
        #     coordinate2 = self.coordinates_from_pos(pos)
        #     print("{} {}".format(pos, coordinate2))

        current_coord = self.coordinates_from_pos(self.start_node.pos)
        current_angle = 0
        print(current_coord)

        while len(self.path) > 1:
            message, current_coord, current_angle = self.get_action(current_coord, current_angle, test=True)
            print(message + " STEPS LEFT:{}".format(len(self.path)))

        return

if __name__ == '__main__':
    path_planner = PathPlanner()

    # will need a map:
    map_maker = MapMaker()
    map_maker.set_map()
    map_maker.calibrate_map()
    lowres_map = map_maker.get_lowres_map()

    path_planner.set_map(lowres_map)
    # path_planner.set_start_node()
    # path_planner.set_end_node()
    # path_planner.set_targets()

    # path_planner.run()
    # path_planner.debug()
    # path_planner.test_action()
    path_planner.test_rover_movement()

    # path_planner.ser.close()