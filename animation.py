import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
import numpy as np

# Local libraries
from vars import MAP_SIZE,SIM_TIME
from simulation import Simulation

def animateSimulation(simulation: Simulation):
    rectangles = simulation.simMap.getRectangles()
    warehouses = np.array(simulation.simMap.getWarehouses()).transpose()
    drones = simulation.getDrones()

    fig, ax = plt.subplots(figsize=(9, 9))

    # Set the limits of the plot
    ax.set_xlim(0, MAP_SIZE[0])
    ax.set_ylim(0, MAP_SIZE[1])

    # Plot edges
    # edges = simulation.simMap.pathFinder.vis_graph.getEdges().reshape(-1, 2).transpose()
    # ax.plot(*edges, c='y', )
    # Plot buildings
    for rect in rectangles:
        degrees = np.degrees(rect[-1])
        ax.add_patch(Rectangle(xy=rect[0], width=rect[1], height=rect[2], angle=degrees, color='gray'))

    # Plot warehouses
    ax.scatter(warehouses[0], warehouses[1], marker=',', s = 40, c='green', label='Warehouses')

    drone_scatters = [ax.scatter(drone.position[0], drone.position[1], marker='v',s=10, c='red', label='Drones') for drone in drones]
    pending_orders = ax.scatter([], [], s = 10, c='yellow', label = 'Pending Orders')
    processed_orders = ax.scatter([], [], s = 10, c='orange', label = 'Processed Orders')

    


    # Update function for animation
    def update(frame):
        simulation.env.step()
        pending = [xy for _, xy, _ in simulation.pending_orders]
        processed = []
        for i, scatter in enumerate(drone_scatters):
            scatter.set_offsets(drones[i].position)
            if drones[i].haveOrder():
                processed.append(drones[i].path[-1])
        pending_orders.set_offsets(pending if pending else np.empty((0, 2)))
        processed_orders.set_offsets(processed if processed else np.empty((0, 2)))
        return drone_scatters + [pending_orders, processed_orders]

    ani = FuncAnimation(fig, update, frames=SIM_TIME, interval=50, blit=True)

    plt.show()