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

TODO:
NOTE: WILL NEED TO CHANGE INITILIAZATIONS FOR COMPATIBILITY WITH MAIN

- return action & arc
        - example actions: "GET UNSTUCK" "FOLLOW PATH"
        - Might only want to pass arc when it detects it has reached the end of the old path

- Compare location to expected location
- create metrics for movement
        - What I thnk is happening based on loction history and when to intervene
- re-format map to slope and set go/no go areas-- CHANGE THE MAP-MAKER
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

class Node():
    def __init__(self, pos=None, parent=None, g=None, h=None):
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
"""

class PathPlanner():
    def __init__(self):
        self.map = []
        self.start_node = None
        self.end_node = None

    '''
    Function: set_map
    Inputs: map (ndarray)
    Default: None
    Returns: None
    Calls: None
    Notes: Sets global variable 'self.map'
    '''

    def set_map(self, map):
        print("PathPlanner: RECEIVED MAP")
        self.map = map
        return

    '''
    Function: set_start_node
    Inputs: start_node (Node)
    Default: start_node = node with position 0,0
    Returns: None
    Calls: None
    Notes: Sets global variable 'start_node'
    '''

    def set_start_node(self, start_node=Node(pos=[0,0])):
        print("PathPlanner: SETTING START NODE")
        self.start_node = start_node
        return

    '''
    Function: set_end_node
    Inputs: end_node (Node)
    Default: end_node = node with position 10,10
    Returns: None
    Calls: None
    Notes: Sets global variable 'end_node'
    '''

    def set_end_node(self, end_node=Node(pos=[10,10])):
        print("PathPlanner: SETTING END NODE")
        self.end_node = end_node
        return

    '''
    Function: calculate_h
    Inputs: pos (tuple)
    Default: None
    Returns: h (distance from current node to end node)
    Calls: self.end_node
    Notes: Returns cost from current node to goal
    '''

    def calculate_h(self, pos):
        # calculate distance from current node to end node
        x = pos[1] # column index
        y = pos[0] # row index

        end_x = self.end_node.pos[1]
        end_y = self.end_node.pos[0]

        h = m.sqrt((x-end_x)**2 + (y-end_y)**2) * 500 # simple triangle dist formula to end: a^2 + b^2 = c^2
        return h

    '''
    Function: calculate_g
    Inputs: positions (list of position tuples)
    Default: None
    Returns: total_g (list of g cost for each pos given)
    Calls: self.map
    Notes: cost currently calculated based on slop
    TODO: modify to correct cost calculations
    '''

    def calculate_g(self, positions):
        """
        Takes list of neighbors and returns an array of weights to match them (the first weight is for the first neighbor, second to the second, etc.)
        calculates g cost with combo of 
        """
        # Generic g cost from current to neighboring points
        generic_cost = [1.4, 1, 1.4,
                1, 1,
                1.4, 1, 1.4]

        # add weight of value (slope) in array
        slope_weight = []
        ##print(positions)

        for i in range(0,len(positions)):
            current_pos = positions[i]
            if current_pos: # If the position exists
                slope = self.map[current_pos[0],current_pos[1]]
                slope_weight.append(slope)
            else:
                slope_weight.append(0)

        total_g = np.add(generic_cost,slope_weight)
        ##print(total_g)
        return total_g

    '''
    Function: get_neighbors
    Inputs: node (Node)
    Default: None
    Returns: neighbors (list of Nodes)
    Calls: self.map
    Notes: Returns positions of currect node's neighbors in 1d array from top left to bottom right. Deals with corner, edge, and general cases.
    '''

    def get_neighbors(self, node):
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

    '''
    Function: plan_path
    Inputs: None
    Default: plot_path = False (won't plot path in progress)
    Returns: path (list of Nodes from start to end)
    Calls: se;f.initialize_plot(), self.start_node, self.get_neighbors(), self.calculate_g(), self.calculate_h(), self.update_plot(), self.end_node
    Notes: Plans a path from self.start_node to self.end_node using A*
    '''

    def plan_path(self, plot_path=False):
        """
        This plans a path using A*
        It calls get_neighbors to obtain the neighbors of the current node
        It calls calculate_g_cost to obtain the g cost for the neighboring points.
        """

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

        print("END PATH PLANNING")
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
        return path

    '''
    Function: initialize_plot
    Inputs: None
    Default: None
    Returns: None
    Calls: self.map, self.start_node, self.end_node
    Notes: Makes heat map of map with start and end nodes overlaid. The path is later drawn over this.
    '''

    def initialize_plot(self):
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

    '''
    Function: update_plot
    Inputs: current_pos, neighbors_pos
    Default: None
    Returns: None
    Calls: None
    Notes: Overlays scatter plot with current node and neighbors over initialized plot
    '''

    def update_plot(self, current_pos, neighbors_pos):
        """
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

    '''
    Function: plot_final_path
    Inputs: path (list of nodes)
    Default: None
    Returns: None
    Calls: self.map, self.start_node, self.end_node
    Notes: Plots the map with path overlain. Shows start and end nodes and the path in between.
    '''

    def plot_final_path(self, path):
        """
        This plots the heatmap with the path overlain. It shows the start and end node and the path in between.
        """
        print("PLOTTING FINAL PATH")        
        x = []
        y = []
        for pos in path:
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
        path = plt.scatter(x, y, marker='o', s=thickness, color=[1, .5, 0])

        plt.legend((start, end, path), ('Start', 'End', 'Path'))

        plt.show()
        return

    '''
    Function: 
    Inputs:
    Default:
    Returns:
    Calls:
    Notes:
    '''

    def run(self):
        print("BEGIN")
        #TODO: Call plan_path and plot_path
        path = self.plan_path(plot_path=False)
        self.plot_final_path(path)
        print("END")
        #self.plot_path(path)

if __name__ == '__main__':
    path_planner = PathPlanner()

    # will need a map:
    map_maker = MapMaker()
    map_maker.set_map()
    lowres_map = map_maker.get_lowres_map()
    path_planner.set_map(lowres_map)
    
    path_planner.set_start_node()
    path_planner.set_end_node()

    path_planner.run()