import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
from matplotlib import cm
from matplotlib.patches import Circle, Rectangle
from matplotlib.collections import LineCollection
import matplotlib.ticker as ticker


### Shortest path demo

#Load results from txt file
path = np.transpose(np.genfromtxt('frc_shortest_path_demo.txt', delimiter=','))
nodes = np.transpose(np.genfromtxt('frc_shortest_path_demo_nodes.txt', delimiter=','))

#Plot data
fig = plt.figure(figsize=(16,8))
ax = plt.gca()
#ax.set_xlim([-1,12])
#ax.set_ylim([-1,12])
#Nodes of search tree
# new! colored by cost!
# new! lines!
edges = list(zip(zip(nodes[1],nodes[2]), zip(nodes[3],nodes[4])))
lc = LineCollection(edges, array=nodes[0])
ax.add_collection(lc)
#plt.scatter(nodes[1],nodes[2],c=nodes[0],s=5)

# outline
ax.add_patch(Rectangle([0,0], 16.54,8.02, facecolor="none", alpha=1.0, edgecolor="black"))
# red charge station
ax.add_patch(Rectangle([2.98, 1.51], 1.93, 2.44, facecolor="red", alpha=0.7, edgecolor="none"))
# blue charge station
ax.add_patch(Rectangle([11.63, 1.51], 1.93, 2.44, facecolor="blue", alpha=0.7, edgecolor="none"))
# nodes
ax.add_patch(Rectangle([0, 0], 1.43, 5.49, facecolor="red", alpha=0.7, edgecolor="none"))
# opponent community
ax.add_patch(Rectangle([13.18, 0], 3.36, 5.49, facecolor="blue", alpha=0.7, edgecolor="none"))
# opponent loading
ax.add_patch(Rectangle([0,5.49], 3.36, 1.26, facecolor="blue", alpha=0.7, edgecolor="none"))
# opponent loading
ax.add_patch(Rectangle([0, 6.75], 6.71, 1.26, facecolor="blue", alpha=0.7, edgecolor="none"))
# example opponents
ax.add_patch(Rectangle([8.5, 4.5], 1, 1, facecolor="blue", alpha=0.7, edgecolor="none"))
ax.add_patch(Rectangle([6.5, 5.5], 1, 1, facecolor="blue", alpha=0.7, edgecolor="none"))


# ax.add_patch(Circle([3.0, 2.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
# ax.add_patch(Circle([6.0, 8.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
# start
ax.add_patch(Circle([16.179, 6.75],0.25, facecolor="green",alpha=0.3,edgecolor="none"))
# end, note this is not tag pose
ax.add_patch(Circle([1.43, 2.748],0.25, facecolor="green",alpha=0.3,edgecolor="none"))



#Solution from planner
plt.plot(path[0],path[1], linewidth=5)
ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
ax.set_aspect('equal', adjustable='box')

plt.autoscale()
plt.show()
