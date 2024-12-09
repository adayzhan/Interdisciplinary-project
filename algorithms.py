import numpy as np

class PathFinder():
    def __init__(self, polygons: np.ndarray, warehouses: list[tuple[float]]):
        self.polygons = polygons
        self.warehouse = warehouses
    
    # Includes initial position
    def findShortestPath(self, a: tuple[float], b: tuple[float]) -> list[tuple[float]]:
        return [a, b]