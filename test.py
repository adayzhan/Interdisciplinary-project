from algorithms import PathFinder
import numpy as np
import matplotlib.pyplot as plt

warehouse = np.array([[15, 20]]) # (n, 2)
polygons = np.array([[[10, 10],[10, 15],[15, 10],[15, 15]]]) # (n, 4, 2)
p = PathFinder(polygons, warehouse, )
fig, ax = plt.subplots(figsize=(3, 3))
edges = p.vis_graph.getEdges().reshape(-1, 2).transpose()
ax.set_xlim(0, 50)
ax.set_ylim(0, 50)
ax.plot(*edges, c='y', )
ax.scatter(*polygons.transpose())
# segments
ax.plot(*edges, c='k')
# identify points: a in blue, b in red
plt.show()