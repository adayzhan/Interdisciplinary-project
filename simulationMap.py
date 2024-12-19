import numpy as np
from numpy import random

# Local libraries
from algorithms import PathFinder
import vars    

class SimulationMap:
    def __init__(self):
        # Map variables
        self.map_size: tuple[float, float] = vars.MAP_SIZE

        # Building variables
        self.build_area_constr: int = vars.BUILD_AREA_CONSTR
        self.build_ratio_constr: int = vars.BUILD_RATIO_CONSTR
        self.num_buildings: int = vars.NUM_BUILDINGS

        # Warehouse variables
        self.num_warehouses: int = vars.NUM_WAREHOUSES
        self.warehouse_radius: int = vars.WAREHOUSE_RADIUS_CONSTR

        # Buildings data structures
        self.rectangles: list[tuple[tuple[float, float], float, float, float]] = [] # [((x, y), width, height, angle)] - angle in radians 
        self.polygons: np.ndarray = np.empty(shape=(0, 4, 2))
        self.warehouses: np.ndarray = np.empty(shape=(0, 2))

        # Create random buildings
        self.__createRandomBuildings()

        # Create random warehouses
        # CAUTION: In rare cases may lead to infinite loop
        self.__createRandomWarehouses()

        self.warehouses.flags.writeable = False
        # Create backup random points
        # CAUTION: In rare cases may lead to infinite loop
        self.random_points: np.ndarray = self.__dummyRandomPoints(10)

        # Initialize path finder
        self.pathFinder = PathFinder(self.polygons, self.warehouses)
    
    def __insideRect(self, point: np.ndarray, xy: np.ndarray, width: float, height: float, angle: float) -> bool:
        tanT = np.tan(angle)
        cotT = 1/tanT
        sinT = np.sin(angle)
        cosT = np.cos(angle)
        if cosT == 0:   
            if sinT == 1 and (point[0] > xy[0] or point[0] < xy[0] - height):
                return False
            elif sinT == -1 and (point[0] < xy[0] or point[0] > xy[0] + height):
                return False
        elif cosT > 0:
            if point[1] < tanT * (point[0] - xy[0]) + xy[1]:
                return False
            elif point[1] > tanT * (point[0] - xy[0] + height * sinT) + xy[1] + height * cosT:
                return False
        elif cosT < 0:
            if point[1] > tanT * (point[0] - xy[0]) + xy[1]:
                return False
            elif point[1] < tanT * (point[0] - xy[0] + height * sinT) + xy[1] + height * cosT:
                return False
        if sinT == 0:
            if cosT == 1 and (point[0] < xy[0] or point[0] > xy[0] + width):
                return False
            elif cosT == -1 and (point[0] > xy[0] or point[0] < xy[0] - width):
                return False
        elif sinT > 0:
            if point[1] < -cotT * (point[0] - xy[0]) + xy[1]:
                return False
            elif point[1] > -cotT * (point[0] - xy[0] - width * cosT + height * sinT) + xy[1] + width * sinT + height * cosT:
                return False
        elif sinT < 0:
            if point[1] > -cotT * (point[0] - xy[0]) + xy[1]:
                return False
            elif point[1] < -cotT * (point[0] - xy[0] - width * cosT + height * sinT) + xy[1] + width * sinT + height * cosT:
                return False
        return True

    def __checkCollision(self, polygon: np.ndarray, xy: np.ndarray, width: float, height: float, angle: float) -> bool:
        for p, rect in zip(self.polygons, self.rectangles):
            for i in range(4):
                if self.__insideRect(polygon[i], *rect) or self.__insideRect(p[i], xy, width, height, angle):
                    return False
            if self.__insideRect((polygon[0] - polygon[3])/2, *rect) or self.__insideRect((p[0] - p[3])/2, xy, width, height, angle):
                return False
        return True

    def __getPolygon(self, xy: np.ndarray, width: float, height: float, angle: float) -> np.ndarray:
        return (np.array([[np.cos(angle), -np.sin(angle)],
                            [np.sin(angle), np.cos(angle)]]) @ np.array([[0, 0, width, width],
                                                                         [0, height, 0, height]],
                                                                         dtype=np.double)).transpose() + xy    
    def __notInRect(self, xy: np.ndarray) -> bool:
        for rect in self.rectangles:
            if self.__insideRect(xy, *rect):
                return False
        return True

    def __createRandomBuildings(self):
        areas = random.uniform(low=self.build_area_constr[0], high=self.build_area_constr[1], size=self.num_buildings)
        xy = np.concatenate([random.uniform(low=0, high=self.map_size[0], size=(self.num_buildings, 1)),
                             random.uniform(low=0, high=self.map_size[1], size=(self.num_buildings, 1))], axis=1)
        angles = random.uniform(low=0, high=np.pi/2, size=self.num_buildings)
        for i in range(self.num_buildings):
            height = random.uniform(low=np.sqrt(areas[i]*self.build_ratio_constr),
                                    high=np.sqrt(areas[i]*self.build_ratio_constr))
            width = areas[i]/height
            for _ in range(vars.COLLISION_ATTEMPTS):
                polygon = self.__getPolygon(xy[i], width, height, angles[i])
                if self.__checkCollision(polygon, xy[i], width, height, angles[i]):
                    self.rectangles.append((xy[i], width, height, angles[i]))
                    self.polygons = np.concatenate((self.polygons, np.expand_dims(polygon, axis=0)))
                    break
                else:
                    xy[i][0] = random.uniform(low=0, high=self.map_size[0])
                    xy[i][1] = random.uniform(low=0, high=self.map_size[1])

    # CAUTION: In rare cases may lead to infinite loop
    def __createRandomWarehouses(self):
        flag = True
        while len(self.warehouses) < self.num_warehouses:
            xy = self.__dummyRandomPoints()
            for warehouse in self.warehouses:
                if np.sum((xy - warehouse)**2) <= 4 * vars.WAREHOUSE_RADIUS_CONSTR**2:
                    flag = False
                    break
            if flag:
                self.warehouses = np.concatenate([self.warehouses, xy.reshape(1, 2)], axis=0)
            else: flag = True

    # CAUTION: In rare cases may lead to infinite loop
    def __dummyRandomPoints(self, size: int = 1) -> np.ndarray:
        points = []
        while len(points) < size:
            xy = (np.random.uniform(low = 0, high = self.map_size[0]), np.random.uniform(low = 0, high = self.map_size[1]))
            if self.__notInRect(xy):
                points.append(xy)
                if size == 1: return np.array(xy)
        return np.array(points)

    def distance(self, path: np.ndarray) -> float:
        d = 0
        for i in range(len(path)-1):
            d += np.sqrt(np.sum((path[i][1] - path[i+1][1])**2))
        return d

    def getRandomWarehouse(self) -> np.ndarray:
        return self.warehouses[np.random.randint(low = 0, high=self.num_warehouses)]

    def getRectangles(self) -> list[tuple[tuple[float, float], float, float, float]]:
        return self.rectangles
        
    def getPolygons(self) -> np.ndarray:
        return self.polygons

    def getWarehouses(self) -> np.ndarray:
        return self.warehouses

    def getRandomPoint(self) -> np.ndarray:
        for _ in range(vars.COLLISION_ATTEMPTS):
            xy = (np.random.uniform(low = 0, high = self.map_size[0]), np.random.uniform(low = 0, high = self.map_size[1]))
            if self.__notInRect(xy):
                return np.array(xy)
        return self.random_points[np.random.randint(low=0, high=len(self.random_points))]

    def getPathToNearestWarehouse(self, point: np.ndarray) -> np.ndarray:
        path = None
        d = np.inf
        for warehouse in self.warehouses:
            if np.all(warehouse == point): return [], 0
            p = self.pathFinder.findShortestPath(point, warehouse)
            if self.distance(p) < d:
                path = p
                d = self.distance(path)
        if path is None: return [], 0
        else: return path, d
    
    def getPathToCustomer(self, position: np.ndarray, destination: np.ndarray) -> tuple[np.ndarray, int, float]:
        path = None
        stop = 0
        d = np.inf
        for warehouse in self.warehouses:
            if np.all((position == warehouse) & (destination == warehouse)):
                return [], 0, 0.0
            elif np.all(position == warehouse):
                path = self.pathFinder.findShortestPath(position, destination)
                return path, 0, self.distance(path)
            elif np.all(destination == warehouse):
                path = self.pathFinder.findShortestPath(position, destination)
                return path, len(path) - 1, self.distance(path)
            else:
                p1 = self.pathFinder.findShortestPath(position, warehouse) # n >= 2
                p2 = self.pathFinder.findShortestPath(warehouse, destination) # n >= 2
                p = np.concatenate([p1, p2], axis=0)
                if self.distance(p) < d:
                    path = p
                    stop = len(p1) - 1
                    d = self.distance(path)
        if path is None: return [], 0, 0.0
        else: return path, stop, d

