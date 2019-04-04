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

class Map_Maker():
    def __init__(self):
        # read data from pre-made "map" file
        self.data = pd.read_csv('example_map.txt', sep=", |;", header=None, engine='python')
        # convert to 2d array
        self.data = self.data.as_matrix()

        # size of map
        rows, columns = self.data.shape

        max_length = 10 # length of box in ft
        max_width = 5 # width of box in ft
        resolution = .25 # "pixel" size in ft (one value in map array will account for an area of resolution^2)

        self.new_rows = int(max_length / resolution)
        self.new_columns = int(max_width / resolution)

    def get_map(self):
        return self.data

    def plot_data(self):
        # plot heat map
        sns.heatmap(self.data, cmap="YlGnBu")
        plt.show()

    def sobel_filter_data(self, data):
        # sobel filter is an edge detector, but might work for slopes
        sobeled_img = ndimage.sobel(data)
        return sobeled_img

    def plot_all(self):
        # plot normal and changed data
        # setup matplotlib fig
        f, axes = plt.subplots(2, 2, sharex=True)
        sns.despine(left=True) # what does this do?

        # plot heatmaps
        sns.heatmap(self.data, cmap="YlGnBu", ax=axes[0, 0])
        sns.heatmap(self.sobel_filter_data(self.data), cmap="YlGnBu", ax=axes[0, 1])
        
        new_map = cv2.resize(self.data, (self.new_columns, self.new_rows))
        new_sobel = self.sobel_filter_data(new_map)

        sns.heatmap(new_map, cmap="YlGnBu", ax=axes[1, 0])
        sns.heatmap(new_sobel, cmap="YlGnBu", ax=axes[1, 1])

        plt.show()

    def run(self):
        self.plot_all()
        return

if __name__ == '__main__':
    code = Map_Maker()
    code.run()