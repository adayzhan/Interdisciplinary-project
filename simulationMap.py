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
        self.polygons: np.ndarray = np.empty(shape=(0, 4, 2)) # n x 4 x 2 - numpy array
        self.warehouses: list[tuple[float, float]] = [] # [(x, y)]

        # Create random buildings
        self.__createRandomBuildings()

        # Create random warehouses
        # CAUTION: In rare cases may lead to infinite loop
        self.__createRandomWarehouses()

        # Create backup random points
        # CAUTION: In rare cases may lead to infinite loop
        self.random_points: list[tuple[float, float]] = self.__dummyRandomPoints(10)

        # Initialize path finder
        self.pathFinder = PathFinder(self.polygons, self.warehouses)
    
    def __insideRect(self, point: tuple, xy: tuple[float, float], width: float, height: float, angle: float) -> bool:
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

    def __checkCollision(self, polygon: np.ndarray, xy: tuple[float, float], width: float, height: float, angle: float) -> bool:
        for p, rect in zip(self.polygons, self.rectangles):
            for i in range(4):
                if self.__insideRect(tuple(polygon[i]), *rect) or self.__insideRect(tuple(p[i]), xy, width, height, angle):
                    return False
            if self.__insideRect(tuple((polygon[0] - polygon[3])/2), *rect) or self.__insideRect(tuple((p[0] - p[3])/2), xy, width, height, angle):
                return False
        return True

    def __getPolygon(self, xy: tuple[float, float], width: float, height: float, angle: float) -> np.ndarray:
        polygon = np.array([[np.cos(angle), -np.sin(angle)],
                            [np.sin(angle), np.cos(angle)]]) @ np.array([[0, 0, width, width],
                                                                         [0, height, 0, height]],
                                                                         dtype=np.double)
        polygon[0] += xy[0]
        polygon[1] += xy[1]
        return polygon.transpose() # 4 x 2
    
    def __notInRect(self, xy: tuple[float, float]) -> bool:
        for rect in self.rectangles:
            if self.__insideRect(xy, *rect):
                return False
        return True

    def __createRandomBuildings(self):
        areas = random.uniform(low=self.build_area_constr[0], high=self.build_area_constr[1], size=self.num_buildings)
        xs = random.uniform(low=0, high=self.map_size[0], size=self.num_buildings)
        ys = random.uniform(low=0, high=self.map_size[1], size=self.num_buildings)
        angles = random.uniform(low=0, high=np.pi/2, size=self.num_buildings)
        for i in range(self.num_buildings):
            height = random.uniform(low=np.sqrt(areas[i]*self.build_ratio_constr),
                                    high=np.sqrt(areas[i]*self.build_ratio_constr))
            width = areas[i]/height
            for _ in range(vars.COLLISION_ATTEMPTS):
                polygon = self.__getPolygon((xs[i], ys[i]), width, height, angles[i])
                if self.__checkCollision(polygon, (xs[i], ys[i]), width, height, angles[i]):
                    self.rectangles.append(((xs[i], ys[i]), width, height, angles[i])) # ((x, y), width, height, angle)
                    self.polygons = np.concatenate((self.polygons, np.expand_dims(polygon, axis=0)))
                    break
                else:
                    xs[i] = random.uniform(low=0, high=self.map_size[0])
                    ys[i] = random.uniform(low=0, high=self.map_size[0])

    # CAUTION: In rare cases may lead to infinite loop
    def __createRandomWarehouses(self):
        flag = True
        while len(self.warehouses) < self.num_warehouses:
            xy = self.__dummyRandomPoints()
            for warehouse in self.warehouses:
                if (xy[0] - warehouse[0])**2 + (xy[1] - warehouse[1])**2 <= 4 * vars.WAREHOUSE_RADIUS_CONSTR**2:
                    flag = False
                    break
            if flag:
                self.warehouses.append(xy)
            else: flag = True

    # CAUTION: In rare cases may lead to infinite loop
    def __dummyRandomPoints(self, size: int = 1) -> list[tuple[float, float]] | tuple[float, float]:
        points = []
        while len(points) < size:
            xy = (np.random.uniform(low = 0, high = self.map_size[0]), np.random.uniform(low = 0, high = self.map_size[1]))
            if self.__notInRect(xy):
                points.append(xy)
        if size == 1: return points[0]
        else: return points

    def distance(self, path: list[tuple[float, float]]) -> int:
        d = 0
        for i in range(len(path)-1):
            d += np.sqrt((path[i][0] - path[i+1][0])**2 + (path[i][1] - path[i+1][1])**2)
        return d

    def getRandomWarehouse(self) -> tuple[float, float]:
        return self.warehouses[np.random.randint(self.num_warehouses)]

    def getRectangles(self) -> list[tuple[tuple[float, float] | float]]:
        return self.rectangles
        
    def getPolygons(self) -> np.ndarray:
        return self.polygons

    def getWarehouses(self) -> list[tuple[float, float]]:
        return self.warehouses

    def getRandomPoint(self) -> tuple[float, float]:
        for _ in range(vars.COLLISION_ATTEMPTS):
            xy = (np.random.uniform(low = 0, high = self.map_size[0]), np.random.uniform(low = 0, high = self.map_size[1]))
            if self.__notInRect(xy):
                return xy
        return self.random_points[np.random.randint(len(self.random_points))]

    def getPathToNearestWarehouse(self, point: tuple[float, float]) -> tuple[list[tuple[float, float]] | list, float]:
        path = []
        d = np.inf
        for warehouse in self.warehouses:
            if warehouse == point: return [], 0
            p = self.pathFinder.findShortestPath(point, warehouse)
            if self.distance(p) < d:
                path = p
                d = self.distance(path)
        return path, d
    
    def getPathToCustomer(self, position: tuple[float, float], destination: tuple[float, float]) -> tuple[list[tuple[float, float]] | list, int, float]:
        path = []
        stop = 0
        d = np.inf
        flags = [True, True]
        for warehouse in self.warehouses:
            if position == warehouse and destination == warehouse:
                return [], 0
            elif position == warehouse:
                stop = 0
                flags[0] = False
            elif destination == warehouse:
                stop = -1
                flags[1] = False
            p1 = self.pathFinder.findShortestPath(position, warehouse) if flags[0] else [position]
            p2 = self.pathFinder.findShortestPath(warehouse, destination) if flags[1] else [destination]
            if self.distance(p1 + p2[1:]) < d:
                path = p1 + p2[1:]
                stop = len(p1) - 1
                d = self.distance(path)
        return path, stop, d
