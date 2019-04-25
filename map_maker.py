"""
takes an opencv map (mat file) and makes 2d map
IMPORTANT NOTE: Had to do weird thing to bash file to get opencv to work. may not run ros in this directory now. https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv
UPDATE: That didn't work at all. No idea what I deleted. currently trying to install opencv differently here; https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/


TODO:
take map (np array) as input and return prettied-up version
    - for now, it's initialized with a pre-existing map

Steps to modify map:
- lower resolution
- reformat to slope map
- set go/no go areas

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
    Default: loads box_image map
    Returns: nothing

    Sets the global variable "self.data" to new map values. Necessary to set before processing any new map with other functions.
    '''
    def set_map(self, map_data = np.loadtxt("example_maps/table_image.txt")):
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
    def get_lowres_map(self, box_len=10, box_width=5, resolution=.25, keep_depth_map=False):
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
        if self.data != []:
            # plot normal and changed data
            # setup matplotlib fig
            f, axes = plt.subplots(2, 2)
            sns.despine(left=True) # what does this do?

            # plot heatmaps of full res and full res edge detected data
            sns.heatmap(self.data, cmap="YlGnBu", ax=axes[0, 0], vmin=0, vmax=2000)
            sns.heatmap(self.sobel_filter_data(self.data), cmap="YlGnBu", ax=axes[0, 1], vmin=0, vmax=5000)
            
            # get lower res maps
            new_map, new_sobel = self.get_lowres_map(keep_depth_map=True)

            # plot heatmaps of lower res maps (reg and sobel)
            sns.heatmap(new_map, cmap="YlGnBu", ax=axes[1, 0], vmin=0, vmax=1000)
            sns.heatmap(new_sobel, cmap="YlGnBu", ax=axes[1, 1], vmin=0, vmax=2000)

            plt.show()
        else:
            print("[MapMaker] Waiting for data.")

    '''
    Function: run
    Inputs: none
    Returns: nothing

    If map_maker is run, processes a test file (default in set_map) and plots 4 heatmaps
    '''
    def run(self):
        self.set_map()
        self.plot_all()
        return

if __name__ == '__main__':
    node = MapMaker()
    node.run()