"""
modified from:
https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2


resources:
https://www.youtube.com/watch?v=pKnV6ViDpAI
https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
https://www.redblobgames.com/pathfinding/a-star/introduction.html

"""
"""
STATUS:
in path_planner, working on whether neighbors exist in open or closed
"""

from collections import deque
import math as m
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import random

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
        # map of costs (2d array)
        self.data = np.array([[0, 1, 2, 3],
                    [4, 5, 6, 7],
                    [8, 9, 10, 11],
                    [1,1,1,1],
                    [1,2,3,4]])

        # creating end node with pos, g, and h costs (no parent yet)
        rows, columns = self.data.shape
        self.end_x = columns-1
        self.end_y = rows-1
        self.end_node = Node(pos=[self.end_y,self.end_x], g=0, h=0) # pos column/rows, y/x are reversed for ease of calucating h later. TODO: make more clear

        # create start node with pos, g, and h costs (no parent ever)
        self.start_x = random.randint(0,columns-1)
        self.start_y = random.randint(0,rows-1)
        self.start_node = Node(pos=[self.start_y,self.start_x])

    def calculate_h(self, pos):
        # calculate distance from current node to end node
        x = pos[1] # column index
        y = pos[0] # row index
        h = m.sqrt((x-self.end_x)**2 + (y-self.end_y)**2) # simple triangle dist formula: a^2 + b^2 = c^2
        return h

    def get_neighbors(self, node):
        # return positions of current node's neighbors
        neighbors = []
        row = node.pos[0]
        col = node.pos[1]

        # get max dimensions of map for corner cases
        max_row, max_column = self.data.shape

        # corner cases
        if (node.pos == [0,0]):
            # top left
            print("NEIGHBORS: TOP LEFT")
            neighbors = [None, None, None,
                        None, [row, col+1],
                        None, [row+1, col], [row+1, col+1]]

        elif (node.pos == [0,max_column-1]):
             # top right
            print("NEIGHBORS: TOP RIGHT")
            neighbors = [None, None, None,
                        [row, col-1], None,
                        [row+1, col-1], [row+1, col], None]

        elif (node.pos == [max_row-1,0]): 
            # bottom left
            print("NEIGHBORS: BOTTOM LEFT")
            neighbors = [None, [row-1, col], [row-1, col+1],
                        None, [row, col+1],
                        None, None, None]

        elif (node.pos == [max_row-1,max_column-1]): 
            # bottom right
            print("NEIGHBORS: BOTTOM RIGHT")
            neighbors = [[row-1, col-1], [row-1, col], None,
                        [row, col-1], None,
                        None, None, None]

        else: # general case
            print("NEIGHBORS: GENERAL CASE")
            neighbors = [[row-1, col-1], [row-1, col], [row-1, col+1],
                        [row, col-1], [row, col+1],
                        [row+1, col-1], [row+1, col], [row+1, col+1]]

        return neighbors

    def plan_path(self):
        # INITIALIZE NODE SETS
        open_set = [] # contains unvisited nodes in cost order by f (lowest to the left)
        created_set = [] # contains all created nodes

        # General g cost from current to neighboring points
        g_cost = [1.4, 1, 1.4,
                1, 1,
                1.4, 1, 1.4]

        # INITIALIZE PATH WITH START NODE AND ITS NEIGHBORS
        current_node = self.start_node
        created_set.append(current_node)
        neighbors_pos = self.get_neighbors(current_node)
        for i in range(0, len(neighbors_pos)):
            if neighbors_pos[i]:
                # create neighbor node
                parent = current_node
                pos = neighbors_pos[i]
                g = g_cost[i]
                h = self.calculate_h(pos)
                new_node = Node(pos=pos, parent=parent, g=g, h=h)

                # add neighbor to open_set
                open_set.append(new_node)
                created_set.append(new_node)

        open_set = sorted(open_set, key=Node.get_f)
        
        # PLAN PATH
        steps = 0
        while (len(open_set) > 0):
            current_node = open_set[0]
            # remove current node from open set
            open_set.pop(0)

            if (current_node == self.end_node):
                break

            neighbors_pos = self.get_neighbors(current_node)

            for i in range(0, len(neighbors_pos)):
                if neighbors_pos[i]:
                    # create node with pos only for comparison
                    pos = neighbors_pos[i]
                    new_node = Node(pos=pos)

                    # check if already created
                    if any(node == new_node for node in created_set):
                        print("NODE ALREADY EXISTS")
                        # check if in open set
                        if any(node == new_node for node in open_set):
                            print("NODE IS IN OPEN SET")
                            # get old node
                            index = open_set.index(new_node)
                            #old_node = open_set[index]
                            print("Node before change:")
                            open_set[index].print_attributes()

                            # compare past and current g cost
                            old_g = open_set[index].g
                            new_g = current_node.g + g_cost[i]

                            # update the parent if the new g cost is lower. Otherwise, leave alone
                            if (old_g > new_g):
                                print("UPDATING NODE")
                                open_set[index].parent = current_node
                                open_set[index].g = new_g

                            print("Node after change:")
                            open_set[index].print_attributes()
                    
                    # if it isn't created, create it
                    else:
                        new_node.parent = current_node
                        new_node.g = g_cost[i]
                        new_node.h = self.calculate_h(pos)
                        open_set.append(new_node)
                        created_set.append(new_node)
            steps = steps+1
            # sort open set again
            open_set = sorted(open_set, key=Node.get_f)

        # curious to see how many steps it took
        print("STEPS TO END: {}".format(steps))
        path = []
        while (current_node != self.start_node):
            path.append(current_node.pos)
            current_node = current_node.parent
        path.append(current_node.pos) # add start node
        
        # reverse so it's from start to end
        path = path[::-1]
        return path

    def plot_path(self, path):
        x = []
        y = []
        for pos in path:
            x.append(pos[1]+.5) # offset to look good on heat map
            y.append(pos[0]+.5)

        # plot heat map
        sns.heatmap(self.data)

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

        """
        # plot first neighbors
        neighbors = self.get_neighbors(self.start_node)
        x = []
        y = []
        for neighbor in neighbors:
            if neighbor:
                x.append(neighbor[1]+.15)
                y.append(neighbor[0]+.15)

        neighbors = plt.scatter(x, y, marker = 'o', s = 100)

        plt.legend((start, end, path, neighbors), ('Start', 'End', 'Path', 'First neighbors'))
        """
        # will need to comment this out if you choose to plot neighbors
        plt.legend((start, end, path), ('Start', 'End', 'Path'))

        plt.show()

    def run(self):
        print("BEGIN")
        #TODO: Call plan_path and plot_path
        path = self.plan_path()
        print("END PATH PLANNING")
        self.plot_path(path)
        """
        open_set = self.plan_path()
        for node in open_set:
            print(node.pos, node.get_f())
            #return
        
        print("this")
        print(open_set[0].get_f())
        """

if __name__ == '__main__':
    code = PathPlanner()
    code.run()