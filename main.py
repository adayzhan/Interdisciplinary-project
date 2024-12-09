import simpy

# Local libraries
from simulation import Simulation
from vars import SIM_TIME
from animation import animateSimulation

if __name__ == "__main__":
    env = simpy.Environment()
    mainSim = Simulation(env)

    # Animate simulation
    animateSimulation(mainSim)

