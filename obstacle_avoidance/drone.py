import time 
import math
#SUPER CLASS
class DroneObstacle: 
    def __init__(self):
        self.drone_id = None
        self.altitude = None
        self.latitude = None
        self.longtitude = None
        self.velocity = None

# Drone
class Drone(DroneObstacle):
    def __init__(self):
        super().__init__()
        self.node = None
        self.grid = None
      
    def set_altitude(self,altitude: int):
        self.altitude = altitude
        self.node = self.grid.get_node(self.altitude) # DEBUG HERE:
        self.node.drone_on_me = True
        return self
    
    def set_grid(self,grid):
        self.grid = grid
        return self
    def get_altitude(self):
        return self.altitude
        
        
    def move_to_altitude(self,target_altitude: int, min_speed = 0.1, damping = 0.2): #move_to specific altitude
        distance = target_altitude-self.altitude
        if abs(distance) < min_speed:
            return True
        step_size = damping * distance  
        self.velocity = step_size
        time.sleep(1)
        self.altitude += self.velocity
        return False

    def move_to_node(self,node): #fsdf
        while not self.move_to_altitude(node.get_altitude()):
            print("NODE ALTITUDE", node.get_altitude())
            print(self.altitude)
        
    def autonomous_steering(self):
        if self.node.obstacle_on_me!=False:
            new_node = self.grid.get_lowest_cost_node(self.node)
            self.move_to_node(new_node)
            self.node = new_node
        else:
           print("DRONE AT ALTITUDE "+ str(self.altitude)+" NO OTHER OBSTACLE")

# ENEMY CLASS
class EnemyDrone(DroneObstacle):
    def __init__(self):
        super().__init__()
        self.remote_id_data = None
    def initialize_drone(self,remote_id_data):
        self.remote_id_data = remote_id_data
        self.drone_id = remote_id_data['drone_id']
        return self
    def to_dict(self):
        return {
            "drone_id": self.drone_id,
            "remote_id_data": self.remote_id_data
        }
    def update_data(self, remote_id_data):
        self.remote_id_data = remote_id_data
        


