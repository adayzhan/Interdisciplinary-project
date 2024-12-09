from enum import Enum
import simpy
import numpy as np
from numpy import random

# Local libraries
from simulationMap import SimulationMap
import vars

class Status(Enum):
    DELIVERING = 0 # devivering a product
    CHARGING = 1 # less than 20% battery
    READY = 2 # more than 20% battery

def calculateTime(distance: float, speed: float, delivery: bool = False) -> float:
    time = distance / speed
    if delivery:
        time += vars.LOADING_TIME + vars.UNLOADING_TIME
    return time

class Drone:
    def __init__(self, env, name: str, position: tuple[float, float], simMap: SimulationMap):
        self.env = env
        self.sleep = env.event()
        self.name: str = name
        self.position: tuple[float, float] = position
        self.simMap: SimulationMap = simMap
        self.battery: float = vars.BATTERY_CAPACITY
        self.speed: float = vars.DRONE_SPEED
        self.status: Status = Status.READY
        self.path = []
        self.stop = 0
        self.order_id = 0
        self.at_station: bool = True
        self.action = env.process(self.run())

    def run(self):
        while True:
            match self.status:
                case Status.READY:
                    try:
                        if self.at_station:
                            yield self.env.process(self.charge())
                            yield self.sleep
                        else:
                            path, _ = self.simMap.getPathToNearestWarehouse(self.position)
                            print(path)
                            yield self.env.process(self.fly(path))
                            self.at_station = True
                    except simpy.Interrupt:
                        continue
                case Status.DELIVERING:
                    self.at_station = False
                    print(f'{self.name} starts journey to process Order {self.order_id}')
                    print(f'{self.name} now at {self.position} 1')
                    yield self.env.process(self.fly(self.path))
                    print(f'{self.name} now at {self.position} 2')
                    #self.fly(self.path[:self.stop + 1])
                    #self.env.timeout(vars.LOADING_TIME)
                    #self.fly(self.path[self.stop:])
                    #self.env.timeout(vars.UNLOADING_TIME)
                    if self.battery < vars.LOW_BATTERY: self.status = Status.CHARGING
                    else: self.status = Status.READY
                case Status.CHARGING:
                    if self.at_station:
                        yield self.env.process(self.charge(vars.LOW_BATTERY))
                        self.status = self.status.READY
                    else:
                        path, _ = self.simMap.getPathToNearestWarehouse(self.position)
                        yield self.env.process(self.fly(path))
                        self.at_station = True

    def fly(self, path):
        if path:
            for p in path[1:]:
                while self.simMap.distance([self.position, p]) > self.speed:
                    pos = list(self.position)
                    pos[0] += (p[0] - pos[0]) / self.simMap.distance([pos, p]) * self.speed
                    pos[1] += (p[1] - pos[1]) / self.simMap.distance([pos, p]) * self.speed
                    self.position = tuple(pos)
                    self.battery -= vars.DISCHARGING_SPEED
                    yield self.env.timeout(1)
                self.position = p

    def charge(self, charge_to=vars.BATTERY_CAPACITY):
        time = (charge_to - self.battery)/vars.CHARGING_SPEED
        if time > 0:
            yield self.env.timeout(time)

    def haveOrder(self) -> bool: return self.status == Status.DELIVERING

class OrderGenerator:
    def __init__(self, env):
        self.env = env
        self.counter = 0
        self.order_received = env.event()
        self.new_orders = []
        self.action = env.process(self.run())
    
    def run(self):
        while True:
            yield self.env.timeout(1)
            num_orders = np.random.poisson(lam = vars.LAMBDA)
            if num_orders > 0:
                self.new_orders = [(self.counter + i, self.env.now) for i in range(num_orders)] # (id, creation_time)
                self.counter += num_orders
                self.order_received.succeed()
                self.order_received = self.env.event()
    
class Simulation:
    def __init__(self, env):
        self.env = env
        self.simMap: SimulationMap = SimulationMap()
        self.orderGenerator: OrderGenerator = OrderGenerator(env)

        # (id, creation_time, processed_time, closed_time)
        self.closed_orders: np.ndarray = np.empty(shape=(0, 4))  
        # (id, (x, y), creation_time)
        self.pending_orders: list[tuple[int, tuple[float, float], float]] = []
        
        self.action = env.process(self.run())

        self.drones: list[Drone] = [Drone(env, f'Drone {i}', self.simMap.getRandomWarehouse(), self.simMap) for i in range(vars.NUM_DRONES)]

    def run(self):
        while True:
            yield self.orderGenerator.order_received
            for id, t in self.orderGenerator.new_orders:
                print(f'Order {id} is received at {t}')
            self.pending_orders.extend([(id, self.simMap.getRandomPoint(), t) for id, t in self.orderGenerator.new_orders])
            pending = []
            for id, xy, t in self.pending_orders:
                selected = None
                p = []
                s = 0
                d = np.inf
                for drone in self.drones:
                    match drone.status:
                        case Status.READY:
                            path, stop, distance = self.simMap.getPathToCustomer(drone.position, xy)
                            if distance < d and drone.battery > vars.DISCHARGING_SPEED * calculateTime(distance, drone.speed, True):
                                selected = drone
                                p = path
                                s = stop
                        case Status.CHARGING:
                            pass
                        case Status.DELIVERING:
                            pass
                if selected:
                    print(f'{selected.name} was selected to process Order {id} at {self.env.now}')
                    selected.order_id = id
                    selected.path = p
                    selected.stop = s
                    self.closed_orders = np.concatenate((self.closed_orders,
                                                         [[id, t, self.env.now,
                                                           self.env.now+calculateTime(distance, drone.speed, True)]]),
                                                        axis=0)
                    selected.status = Status.DELIVERING
                    selected.action.interrupt()
                else: pending.append((id, xy, t))
            self.pending_orders = pending

    def getDrones(self):
        return self.drones