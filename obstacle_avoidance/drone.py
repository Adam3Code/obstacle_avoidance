import time 
import math
import sys

#SUPER CLASS
class DroneObstacle: 
    def __init__(self):
        self.drone_id = None
        self.altitude = None
        self.latitude = None
        self.longtitude = None
        self.x_vel = None
        self.y_vel = None
        self.z_vel = None
        self.node = None
    # getters
    def get_id(self)->str:
        return self.drone_id
    def get_node(self):  # Specify the return type
        return self.node
    def get_altitude(self) -> int:
        return self.altitude
    def set_node(self,node):
        self.node = node

    
    #setters
    def set_id(self,drone_id: str):
        self.drone_id = drone_id
    

# Drone
class Drone(DroneObstacle):
    def __init__(self):
        super().__init__()
        self.grid = None
    #setters
    def set_altitude(self,altitude: int):
        self.altitude = altitude
        self.node = self.grid.get_node(self.altitude) # DEBUG HERE:
        self.node.drone_on_me = True
        return self
    
    def set_grid(self,grid):
        self.grid = grid
        return self
    

    #movement
    def move_to_altitude(self,target_altitude: int, min_speed = 0.1, damping = 0.2): #move_to specific altitude
        distance = target_altitude-self.altitude
        step_size = damping * distance  
        self.velocity = step_size
        time.sleep(1)
        self.altitude += self.velocity

    def move_to_node(self,node): #fsdf
        self.move_to_altitude(node.get_altitude())
    
    #############DELETE MAYBE############# 
    def autonomous_steering(self):
        if len(self.node.drones_on_me)>1:
            self.node.set_cost(1000)
            print("DRONE AT ALTITUDE "+ str(self.altitude)+" DRONE ON ME")
        else:
            print("DRONE AT ALTITUDE "+ str(self.altitude)+" NO DRONE ON ME")
        new_node = self.grid.get_lowest_cost_node(self.node)
        self.move_to_node(new_node)

# ENEMY CLASS
class EnemyDrone(DroneObstacle):
    def __init__(self):
        super().__init__()
        self.remote_id_data = None
    def update_data(self,remote_id_data):
        self.altitude = remote_id_data['location']['altitude']
        self.latitude = remote_id_data['location']['latitude']
        self.longitude = remote_id_data['location']['longitude']  
        self.drone_id = remote_id_data['drone_id']
        return self