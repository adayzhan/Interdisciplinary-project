import numpy as np
import copy
from dijkstar import Graph, find_path

class VisGraph(Graph):
    def __init__(self):
        super().__init__(undirected=True)
        self.points_to_keys: list[np.ndarray, int] = [] # (xy, key)
        self.keys_to_points: dict[int: np.ndarray] = {} # key: xy
        self.counter = 0

    def __search_key(self, p: np.ndarray) -> int:
        low = 0
        high = len(self.points_to_keys) - 1
        mid = 0
        while low <= high:
            mid = (high + low) // 2
            if self.points_to_keys[mid][0][0] < p[0]:
                low = mid + 1
            elif self.points_to_keys[mid][0][0] > p[0]:
                high = high - 1
            elif self.points_to_keys[mid][0][1] < p[1]:
                low = mid + 1
            elif self.points_to_keys[mid][0][1] > p[1]:
                high = high - 1
            else:
                return self.points_to_keys[mid][1]
        return -1

    def __insert(self, p: np.ndarray, key: int):
        low = 0
        high = len(self.points_to_keys) - 1
        mid = 0
        while low <= high:
            mid = (high + low) // 2
            if self.points_to_keys[mid][0][0] < p[0]:
                low = mid + 1
            elif self.points_to_keys[mid][0][0] > p[0]:
                high = high - 1
            elif self.points_to_keys[mid][0][1] < p[1]:
                low = mid + 1
            elif self.points_to_keys[mid][0][1] >= p[1]:
                high = high - 1
        self.points_to_keys.insert(mid, (p, key))
        self.keys_to_points[key] = p

    def point_exists(self, p: np.ndarray):
        return self.__search_key(p) >= 0

    def add_edge(self, a: np.ndarray, b: np.ndarray, weight: float):
        u = self.__search_key(a)
        v = self.__search_key(b)

        if u < 0:
            u = self.counter
            self.__insert(a, u)
            self.counter += 1

        if v < 0:
            v = self.counter
            self.__insert(b, v)
            self.counter += 1

        super().add_edge(u, v, weight)
    
    def find_shortest_path(self, a: np.ndarray, b: np.ndarray):
        u = self.__search_key(a)
        v = self.__search_key(b)

        if u < 0 or v < 0:
            raise Exception('No such node!!!')
        else:
            nodes = find_path(self, u, v).nodes
            return [self.keys_to_points[key] for key in nodes]
        
    def getEdges(self) -> np.ndarray: # (n, 2, 2)
        d = self.get_data()
        l = []
        for x, v in d.items():
            for y in v:
                if self.keys_to_points[x][0] < self.keys_to_points[y][0] or (self.keys_to_points[x][0] == self.keys_to_points[y][0] and self.keys_to_points[x][1] < self.keys_to_points[y][1]):
                    l.append([self.keys_to_points[x], self.keys_to_points[y]])          
        return np.array(l)
        
class PathFinder():
    def __init__(self, polygons: np.ndarray, warehouses: np.ndarray):
        self.polygons = polygons # (n, 4, 2)
        self.warehouses = warehouses # (m, 2)
        self.vertices = [(polygons[i][j], (i, j)) for i in range(len(polygons)) for j in range(4)] # (n, 2, 2)
        self.vertices.extend([(p, (-1, -1)) for p in self.warehouses])
        self.vis_graph: VisGraph = self.__visibilityGraph()

    def __intersect(self, a: np.ndarray, b: np.ndarray) -> bool: # (2, 2), (2, 2)
       
        h = (a[0][0] - a[1][0]) * (b[0][1] - b[1][1]) - (a[0][1] - a[1][1]) * (b[0][0] - b[1][0])
        if h != 0:
            detA = a[0][0] * a[1][1] - a[0][1] * a[1][0]
            detB = b[0][0] * b[1][1] - b[0][1] * b[1][0]
            p = (detA * (b[0] - b[1]) - detB * (a[0] - a[1]))/h
            
            if np.isclose(a[0][0], a[1][0]) and np.isclose(p[0], a[0][0]):
                return False
            if np.isclose(a[0][0], a[1][0]) and not np.isclose(p[0], a[0][0]):
                return False
            if (a[0][0] > p[0] and p[0] > a[1][0]) or (a[1][0] > p[0] and p[0] > a[0][0]) or (np.isclose(a[0][0], a[1][0]) and np.isclose(p[0], a[0][0])):
                if (a[0][1] > p[1] and p[1] > a[1][1]) or (a[1][1] > p[1] and p[1] > a[0][1]) or (np.isclose(a[0][1], a[1][1]) and np.isclose(p[1], a[0][1])):
                    if (b[0][0] > p[0] and p[0] > b[1][0]) or (b[1][0] > p[0] and p[0] > b[0][0]) or (np.isclose(b[0][0], b[1][0]) and np.isclose(p[0], b[0][0])):
                        if (b[0][1] > p[1] and p[1] > b[1][1]) or (b[1][1] > p[1] and p[1] > b[0][1]) or (np.isclose(b[0][1], b[1][1]) and np.isclose(p[1], b[0][1])):
                            return True
                        else: return False
                    else: return False
                else: return False
            else: return False
        else: return False

    # TODO optimize, works for rectangles
    def __intersectInteriorRect(self, p, i, j) -> bool: # (2,)
        if p in self.polygons[i]:
            if np.all(p == self.polygons[i][3 - j]): return True
            else: return False
        ineq = np.transpose((self.polygons[i] - self.polygons[i][j])*(p - self.polygons[i][j])*(1, -1))
        return np.all(ineq[0] >= ineq[1])

    # a, b - points (2,), T - (n, 2, 2), W - (n, 2, 2), i, j - coors of a polygon 
    def __visible(self, a: np.ndarray, b: np.ndarray, T, W, notVis, i, j) -> bool:
        if np.all(a == b):
            return False
        elif i >= 0 and j >= 0 and self.__intersectInteriorRect(a, i, j):
            return False
        elif len(W) == 1 or np.all((W[i-1][0][1] - a[1])*(a[0] - b[0]) != (W[i-1][0][1] - a[0])*(a[1] - b[1])):
            for m in T:
                if self.__intersect(np.concatenate([a.reshape(1, 2), b.reshape(1, 2)], axis=0), m):
                    return False
            return True
        elif notVis:
            return False
        else:
            for m in T:
                if self.__intersect(np.concatenate([W[i-1][0].reshape(1, 2), b.reshape(1, 2)], axis=0), m):
                    return False
            return True

    def __visibleVertices(self, p):
        # (n, 2, 2) ((x, y), (i, j)) or ((x, y), (-1, -1))
        vertices = copy.deepcopy(self.vertices)
        vertices.sort(key=lambda v: (np.atan2(v[0][1] - p[1], v[0][0] - p[0]), (v[0][0] - p[0]) ** 2 + (v[0][1] - p[1]) ** 2))
        T = [] # (n, 2, 2)
        atan2E = [] # (n,)
        atan2V = [] # (n,)
        vertices = np.array(vertices)
        k = ((1, 2), (0, 3), (0, 3), (1, 2))
        for v in vertices:
            a = np.atan2(v[0][1] - p[1], v[0][0] - p[0])
            atan2V.append(a)
            if np.any(p != v[0]) and v[1][0] >= 0:
                m = self.polygons[int(v[1][0])][k[int(v[1][1])][0]] # (2,)
                n = self.polygons[int(v[1][0])][k[int(v[1][1])][1]] # (2,)
                a = np.atan2(v[0][1] - p[1], v[0][0] - p[0])
                b = np.atan2(m[1] - p[1], m[0] - p[0])
                c = np.atan2(n[1] - p[1], n[0] - p[0])
                if a < b and np.any(p != m):
                    T.append(np.concatenate([v[0].reshape(1, 2), m.reshape(1, 2)]))
                    atan2E.append(a)
                if a < c and np.any(p != n):
                    T.append(np.concatenate([v[0].reshape(1, 2), n.reshape(1, 2)]))
                    atan2E.append(a)
        T = np.array(T)
        W = [] # (n, 2)
        notVis = False
        for i in range(len(vertices)):
            if self.__visible(p, vertices[i][0], T, vertices, notVis, int(vertices[i][1][0]), int(vertices[i][1][1])):
                W.append(vertices[i][0])
                low = 0
                high = len(atan2E) - 1
                mid = 0
                while low <= high:
                    mid = (high + low) // 2
                    if atan2E[mid] < atan2V[i]:
                        low = mid + 1
                    elif atan2E[mid] >= atan2V[i]:
                        high = mid - 1
                T = T[mid:]
                notVis == False
            else: notVis == True
        return np.array(W)

    def __visibilityGraph(self):
        graph = VisGraph()
        for v in np.array(self.vertices):
            W = self.__visibleVertices(v[0])
            print(f'Center point: {v[0]}')
            print(f'Visible points: {W}')
            for w in W:
                graph.add_edge(v[0], w, np.sqrt(np.sum((v[0] - w)**2)))
        return graph

    # Includes initial position
    def findShortestPath(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        fa = self.vis_graph.point_exists(a)
        fb = self.vis_graph.point_exists(b)
        if fa and fb:
            return self.vis_graph.find_shortest_path(a, b)
        graph = copy.deepcopy(self.vis_graph)
        if not fa:
            W = self.__visibleVertices(a)
            for w in W:
                if np.all(a != w):
                    graph.add_edge(a, w, np.sqrt(np.sum((a - w)**2)))
        if not fb:
            W = self.__visibleVertices(b)
            for w in W:
                if np.all(b != w):
                    graph.add_edge(b, w, np.sqrt(np.sum((b - w)**2)))
        return graph.find_shortest_path(a, b)