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
Switch to branch with "main" file

- return action & arc
        - example actions: "GET UNSTUCK" "FOLLOW PATH"
        - Might only want to pass arc when it detects it has reached the end of the old path

- Compare location to expected location
- create metrics for movement
        - What I thnk is happening based on loction history and when to intervene
- re-format map to lower resolution-- CHANGE THE MAP-MAKER
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

class PathPlanner():
    def __init__(self):
        print("LOADING MAP")
        # map of costs (2d array)
        new_map = MapMaker()
        new_map.set_map()
        self.data = new_map.get_lowres_map()

        print("CREATING START AND END NODES")

        # creating end node with pos, g, and h costs (no parent yet)
            # get size parameters:
        rows, columns = self.data.shape

            # Different test cases:
                # random
        #self.end_x = random.randint(0,columns-1)
        #self.end_y = random.randint(0,rows-1)
                # Bottom right corner
        self.end_x = columns-1
        self.end_y = rows-1
                # check obstacle avoidance
        #self.end_x = 20
        #self.end_y = 16
        self.end_node = Node(pos=[self.end_y,self.end_x], g=0, h=0) # pos column/rows, y/x are reversed for ease of calucating h later. TODO: make more clear

        # create start node with pos, g, and h costs (no parent ever)
                # random
        #self.start_x = random.randint(0,columns-1)
        #self.start_y = random.randint(0,rows-1)
                # check obstacle avoidance
        self.start_x = 0
        self.start_y = 2
        self.start_node = Node(pos=[self.start_y,self.start_x])

    def calculate_h(self, pos):
        # calculate distance from current node to end node
        x = pos[1] # column index
        y = pos[0] # row index
        h = m.sqrt((x-self.end_x)**2 + (y-self.end_y)**2) * 500 # simple triangle dist formula to end: a^2 + b^2 = c^2
        return h

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
                slope = self.data[current_pos[0],current_pos[1]]
                slope_weight.append(slope)
            else:
                slope_weight.append(0)

        total_g = np.add(generic_cost,slope_weight)
        ##print(total_g)
        return total_g

    def get_neighbors(self, node):
        """
        Return positions of current node's neighbors in 1d array from top left to bottom right

        This deals with corner and edge cases, and calls all else the general case
        """
        neighbors = []
        row = node.pos[0]
        col = node.pos[1]

        # get max dimensions of map for corner cases
        max_row, max_column = self.data.shape

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

    def plan_path(self):
        """
        This plans a path using A*
        It calls get_neighbors to obtain the neighbors of the current node
        It calls calculate_g_cost to obtain the g cost for the neighboring points.
        """
        print("BEGIN PATH PLANNING")

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

    def initialize_plot(self):
        print("INITIALIZING PLOT")
        sns.heatmap(self.data, cmap="YlGnBu")

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

    def plot_final_path(self, path):
        """
        This plots the heatmap with the path overlain. It shows the start and end node and the path in between. Can currently show the startnode's neighbors if you uncomment that section of code. TOOD: Make "if neighbors=True" option when calling this function
        """
        print("PLOTTING FINAL PATH")
        x = []
        y = []
        for pos in path:
            x.append(pos[1]+.5) # offset to look good on heat map
            y.append(pos[0]+.5)

        # plot heat map
        sns.heatmap(self.data, cmap="YlGnBu")

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

        # will need to comment this out if you choose to plot neighbors
        plt.legend((start, end, path), ('Start', 'End', 'Path'))

        plt.show()

    def run(self):
        print("BEGIN")
        #TODO: Call plan_path and plot_path
        self.initialize_plot()
        path = self.plan_path()
        self.plot_final_path(path)
        print("END")
        #self.plot_path(path)

if __name__ == '__main__':
    code = PathPlanner()
    code.run()