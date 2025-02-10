import numpy as np
from drone import EnemyDrone
from std_msgs.msg import String
import json
from drone import Drone
import rclpy
from rclpy.node import Node as N



drones = {}  # Dictionary to store drones data





class RemoteIDSubscriberNode(N):
    def __init__(self):
        super().__init__('remote_id_subscriber')
        self.subscription = self.create_subscription(
            String,  # Message type
            'remote_id',  # Topic to subscribe to
            self.remote_id_callback,
            10  # QoS profile
        )
        # Publisher for enemy drones data
        self.publisher = self.create_publisher(String, 'enemy_drone_publisher', 10)
        self.get_logger().info("RemoteIDSubscriberNode started.")

    def remote_id_callback(self, msg):
        # Parse the incoming message
        remote_id_data = json.loads(msg.data)
        drone_id = remote_id_data['drone_id']

        # Add new drone or update existing drone
        if drone_id not in drones:
            drones[drone_id] = EnemyDrone().initialize_drone(remote_id_data)
            self.get_logger().info(f"New Drone Added: {drone_id} ,{drones[drone_id]}")
        else:
            self.get_logger().info(f"Drone {drone_id} data updated.")

   

class SLinkedList:
   def __init__(self):
      self.headval = None

class Node:
    def __init__(self, interval,cost=1):
        self.altitude_start = interval['start']
        self.altitude_end =  interval['end']
        self.altitude = self.altitude_start + (self.altitude_end-self.altitude_start)/2
        self.obstacle = None
        self.obstacle_on_me:bool = False
        self.drone_on_me = None
        self.nextval = None # for linked list
        self.prevval = None
        self.cost: int = cost 
    def get_start_altitude(self):
        return self.altitude_start
    def get_altitude_end(self):
        return self.altitude_end
    def get_altitude(self):
        return self.altitude
    def set_obstacle_on_me(self,bool:bool):
        self.obstacle_on_me = bool
    def set_drone(self,drone):
        self.drone_on_me = drone
        return self
    def remove_obstacle(self):
        self.obstacle_on_me = False
    def get_interval(self):
        return self.altitude_start,self.altitude_end
    def get_cost(self):
        return self.cost
    
    
    
class Grid: #1D
    def __init__(self,start=40,end=200,node_range=10):
        self.start = start 
        self.end = end
        self.node_range = 10 
        self.grid = []
        self.nodes = SLinkedList()
        self.drones = []
    def intialize(self,start=40,end=200): # intialize nodes along altitude
        self.start = start + self.node_range # first node is already done, because of head intialization
        self.end = end 
        self.nodes.headval = Node({'start':start,'end':start+self.node_range-1}) # intialize head of linked list
        node = self.nodes.headval # intialize first node
        for i in range(self.start,self.end,self.node_range):
            interval = {'start':i,'end':i+self.node_range-1}
            node.nextval = Node(interval)
            node.nextval.prevval = node
            node = node.nextval
            self.grid.append(node.prevval)
        return self
    def get_node(self,altitude:int): # gets node
        for node in self.grid:
            if node.altitude_start<=altitude and node.altitude_end>=altitude:
                return node
    def get_lowest_cost_node(self,node:Node): #NEEDS FIXING
        prev_node:Node = node.prevval
        next_node:Node = node.nextval
        if prev_node!=None and next_node!=None:
            if prev_node.get_cost()<node.get_cost():
                return prev_node
            elif next_node.get_cost()>=node.get_cost():
                return prev_node
            else:
                print("no movement") # Could implement recursive function, but not necessary since there is only two drones 
                return node
        elif prev_node!=None and next_node==None:
            return prev_node # only reasonable cause there is only one drone
        elif prev_node==None and next_node!=None:
            return next_node # only reasonable cause there is only one drone

    def set_obstacle_on_node(self, altitude:int): # fix it to be node as input
        node: Node = self.get_node(altitude)
        node.set_obstacle_on_me(True)
        return self

    def print_grid(self): # print the grid
        for node in self.grid:
            print(node.get_interval())

        
def main(args=None):
    rclpy.init(args=args)
    subscriber_node = RemoteIDSubscriberNode()
    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()



altitude_inteval = {'start':50,'end':300}
grid = Grid().intialize().set_obstacle_on_node(60)
drone_1 = Drone().set_grid(grid).set_altitude(60)

cycle = 0 
grid.print_grid()

# MAIN-LOOP

while cycle <20:
    drone_1.autonomous_steering()
    cycle +=1
    

