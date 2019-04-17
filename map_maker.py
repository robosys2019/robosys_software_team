"""
This is built to make a one-time map for the full mission. It takes data from the kinect (as a numpy array) and returns a lower res slope map.
***UNITS ARE IN FEET IN THE MAP***

It has the functions:
- set_map (YOU MUST DO THIS to get an accurate lowres map back)
- get_lowres_map (returns a low res slope map from set_map)
- plot_data (plots the raw map given in set_map)
- sobel_filter_data (transforms depth data to slope data)
- plot_all (plots high res map with depth and slope data, then low res map with depth and slope data)
- run
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import ndimage
import cv2
import numpy as np

class MapMaker():
    def __init__(self):
        # initialize np array for map data
        self.data = []

    '''
    Function: set_map
    Inputs: Np array of depth ranges "map_data"
    Default: loads image from example_maps folder. Options:
        (pd.read_csv('example_maps/bad.txt', sep=", |;", header=None, engine='python')).as_matrix()
        np.loadtxt("example_maps/table_image.txt"))
        np.loadtxt("example_maps/box_image.txt"))
    Returns: nothing

    Sets the global variable "self.data" to new map values. Necessary to set before processing any new map with other functions.
    '''
    def set_map(self, map_data = np.loadtxt("example_maps/box_image.txt")):
        self.data = map_data
        return

    '''
    Function: get_new_map
    Inputs: Box length (ft) "box_len", Box width (ft) "box_width", desired block size (ft) "resolution"
    Default: 10x5 map with .25ft resolution
    Returns: Low res map data, either just with slopes or two maps, one with depth and another with slope

    By default returns map with lower resolution and slope data instead of depth data
    If keep_depth_map is True, returns the lowered res map with depth data AND the low res slope map (NECESSARY FOR PLOT_ALL FUNCTION)
    '''
    def get_lowres_map(self, box_len=16, box_width=7.75, resolution=.66, keep_depth_map=False):
        # determine new desired size of rows/columns based on parameters
        new_rows = int(box_len / resolution)
        new_columns = int(box_width / resolution)

        # resize map with new sizes
        resized_map = cv2.resize(self.data, (new_columns, new_rows))
        # run "edge detection" on map for slope data
        sobeled_map = self.sobel_filter_data(resized_map)

        if keep_depth_map:
            return resized_map, sobeled_map
        else:
            return sobeled_map

    '''
    Function: plot_data
    Inputs: None
    Returns: nothing

    plots the full res depth map set by function set_map
    '''
    def plot_data(self):
        # plot heat map
        sns.heatmap(self.data, cmap="YlGnBu")
        plt.show()

    '''
    Function: sobel_filter_data
    Inputs: data
    Returns: Filtered data

    Uses a sobel filter to calculate slopes from depths
    '''
    def sobel_filter_data(self, data):
        # sobel filter is an edge detector, but might work for slopes
        sobeled_img = ndimage.sobel(data)
        return sobeled_img

    '''
    Function: plot_all
    Inputs: None
    Returns: Nothing

    Plots 4 heatmaps of original data (depth and slope) and low res data (depth and slope)
    '''
    def plot_all(self):
        # plot normal and changed data
        # setup matplotlib fig
        f, axes = plt.subplots(2, 2, sharex=True)
        sns.despine(left=True) # what does this do?

        # plot heatmaps of full res and full res edge detected data
        sns.heatmap(self.data, cmap="YlGnBu", ax=axes[0, 0])
        sns.heatmap(self.sobel_filter_data(self.data), cmap="YlGnBu", ax=axes[0, 1])
        
        # get lower res maps
        new_map, new_sobel = self.get_lowres_map(keep_depth_map=True)

        # plot heatmaps of lower res maps (reg and sobel)
        sns.heatmap(new_map, cmap="YlGnBu", ax=axes[1, 0])
        sns.heatmap(new_sobel, cmap="YlGnBu", ax=axes[1, 1])

        plt.show()

    '''
    Function: mess_with_example_map(self):

    for testing purposes. seeing if I can manipulate test data.
    '''
    def calibrate_map(self):
        return

    '''
    Function: run
    Inputs: none
    Returns: nothing

    If map_maker is run, processes a test file (default in set_map) and plots 4 heatmaps
    '''
    def run(self):
        self.set_map()
        #calibrate_map()
        self.plot_all()
        return

if __name__ == '__main__':
    node = MapMaker()
    node.run()