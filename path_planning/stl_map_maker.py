" Takes STL file and turns it into map of slopes for path_planner"
import numpy as np
from stl import mesh
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

class Stl_Map_Maker():
    def __init__(self):
        # load existing stl file
        self.data = mesh.Mesh.from_file('Aristarchusregionmoon3xv.stl')
        print(self.data['vectors'][0])

    def plot_stl(self):
        # create new plot
        figure = plt.figure()
        axes = mplot3d.Axes3D(figure)

        # load STL file and add vectors to plot
        axes.add_collection3d(mplot3d.art3d.Poly3DCollection(self.data.vectors,edgecolor='k'))

        # Auto scale to the mesh size
        scale = self.data.points.flatten(-1)
        axes.auto_scale_xyz(scale, scale, scale)

        # show plot to the screen
        plt.show()

    def test_plot(self):
        return

    def run(self):
        #self.plot_stl()
        return

if __name__ == '__main__':
    code = Stl_Map_Maker()
    code.run()