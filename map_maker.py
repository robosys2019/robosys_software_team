"""
takes an opencv map (mat file) and makes 2d map
Note: I removed the front and end brackets from the file manually (TODO: automate this)
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

class Map_Maker():
    def __init__(self):
        self.data = pd.read_csv('mat.txt', sep=", |;", header=None, engine='python')
        #print(self.data)
        self.data = self.data.as_matrix() # convert to numpy array
        #print(self.data)

    def plot_data(self):
        """
        This plots the heatmap with the path overlain. It shows the start and end node and the path in between. Can currently show the startnode's neighbors if you uncomment that section of code. TOOD: Make "if neighbors=True" option when calling this function
        """

        # plot heat map
        sns.heatmap(self.data, cmap="YlGnBu")

        plt.show()

    def get_map(self):
        return self.data

    def run(self):
        self.plot_data()
        return

if __name__ == '__main__':
    code = Map_Maker()
    code.run()