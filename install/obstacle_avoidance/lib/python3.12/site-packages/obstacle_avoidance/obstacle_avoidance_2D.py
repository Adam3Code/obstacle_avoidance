import heapq
import math

#THIS PYTHON FILE IS NOT BEING USED
directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

class Node:
    def __init__(self, x, y, cost=1):
        self.x = x
        self.y = y
        self.g = math.inf  # cost to get to this node from the start
        self.h = 0         # heuristic (estimated cost to goal)
        self.f = math.inf  # total cost (g + h)
        self.parent = None # parent node for path reconstruction
        self.status = "unvisited" # status: "unvisited", "open", "closed"
        self.cost = cost   # cost of the node (default is 1, or more for obstacles)

    def __lt__(self, other):
        return self.f < other.f


# D* Lite Algorithm
class DStarLite:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.open_list = []  # priority queue
        self.closed_list = set()  # visited nodes
        self.nodes = {}
        self.init_nodes()
        
    def init_nodes(self):
       
        for i in range(len(self.grid)):
            for j in range(len(self.grid[i])):
                self.nodes[(i, j)] = Node(i, j, self.grid[i][j])

    def heuristic(self, node):
        return abs(node.x - self.goal[0]) + abs(node.y - self.goal[1])

    def initialize(self):
      
        self.nodes[self.start].g = 0
        self.nodes[self.start].h = self.heuristic(self.nodes[self.start])
        self.nodes[self.start].f = self.nodes[self.start].g + self.nodes[self.start].h
        self.nodes[self.start].status = "open"
        
     
        heapq.heappush(self.open_list, self.nodes[self.start])

    def update_node(self, node):
       
        node.f = node.g + node.h

    def get_neighbors(self, node):
      
        neighbors = []
        for dx, dy in directions:
            nx, ny = node.x + dx, node.y + dy
            print(nx, ny)
            if 0 <= nx < len(self.grid) and 0 <= ny < len(self.grid[0]):
                if self.grid[nx][ny] == 0:  # if not an obstacle an obstacle
                    neighbors.append(self.nodes[(nx, ny)])
        return neighbors

    def expand(self):
        
        while self.open_list:
            current = heapq.heappop(self.open_list)
            current.status = "closed"
            
            if current == self.nodes[self.goal]:
                #  goal is reached, reconstruction of path
                path = []
                while current.parent:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1]  # Reverse the path

            #  neighbors
            for neighbor in self.get_neighbors(current):
                if neighbor.status == "closed":
                    continue
                new_g = current.g + current.cost
                if new_g < neighbor.g:
                    neighbor.g = new_g
                    neighbor.h = self.heuristic(neighbor)
                    self.update_node(neighbor)
                    neighbor.parent = current

                    if neighbor.status == "unvisited":
                        heapq.heappush(self.open_list, neighbor)
                        neighbor.status = "open"
        
        return None  # no path found
    def print_grid(self):
        for row in self.grid:
            print(row)


    def replan(self, new_obstacle):
     
        x, y = new_obstacle
        self.grid[x][y] = 1 
        self.nodes[(x, y)].cost = math.inf  
        print(f"Obstacle placed at {new_obstacle}. Grid after obstacle:")
        self.print_grid()  # Print the grid after the obstacle is placed


        return self.expand()


# the grid (0 = free, 1 = obstacle)
grid = [
    [0, 0, 0, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0]
]

start = (0, 0)  
goal = (4, 4)  
dstar = DStarLite(grid, start, goal)
dstar.initialize()


path = dstar.expand()
print("Initial Path:", path)


new_obstacle = (1, 2)  # New obstacle location
print("new obstacle", new_obstacle)
path_after_replanning = dstar.replan(new_obstacle)
print("new epath ", path_after_replanning)
