import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib.animation as anim

class MapBuilder:
    def plot_map(self, map_data):
        if map_data != []:
                fig = plt.figure()
                ax = fig.add_subplot(1,1,1)

                def update(i):
                    if i==1:
                        cbar_boolean = True
                    else:
                        cbar_boolean = False
                    ax.clear()
                    sns.heatmap(map_data, cmap="YlGnBu", ax=ax, cbar=cbar_boolean)

                a = anim.FuncAnimation(fig, update, repeat=True)

                plt.show()
        else:
            print("[MapBuilder] Waiting for data.")