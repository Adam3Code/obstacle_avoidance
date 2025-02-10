import sys
sys.path.append('/home/adamrustom/ros2_ws/src/obstacle_avoidance')
import numpy as np
from obstacle_avoidance.drone import EnemyDrone
from std_msgs.msg import String
import json
from obstacle_avoidance.drone import Drone
import rclpy
from rclpy.node import Node as N
import pdb  # Import the pdb module


drones = {}


#intialized linkedlist used for the grid
class SLinkedList:
   def __init__(self):
      self.headval = None
#class Node
class Node:
    def __init__(self, interval,cost=1):
        self.altitude_start = interval['start']
        self.altitude_end =  interval['end']
        self.altitude = self.altitude_start + (self.altitude_end-self.altitude_start)/2
        self.obstacle = None 
        self.drones_on_me = {} # drone id -> drone
        self.obstacle_on_me:bool = False
        self.nextval = None # for linked list
        self.prevval = None
        self.cost: int = cost 
    # getters
    def get_start_altitude(self):
        return self.altitude_start
    def get_altitude_end(self):
        return self.altitude_end
    def get_altitude(self):
        return self.altitude
    def get_interval(self):
        return self.altitude_start,self.altitude_end
    def get_cost(self):
        return self.cost
    #setters
    def set_obstacle_on_me(self,bool:bool):
        self.obstacle_on_me = bool
    def set_cost(self,cost:int):
        self.cost = cost
    #methods
    def add_drone(self,drone:Drone):
        self.drones_on_me[drone.get_id()] = drone
    def remove_drone(self,drone:Drone):
        drone_id = drone.get_id()
        if drone_id in self.drones_on_me:
            del self.drones_on_me[drone_id] # make sure it doesn't remove the actual object, but it remove the object from the dictionary
    #DELETE MAYBE#
    def remove_obstacle(self):
        self.obstacle_on_me = False
    
    
    
class Grid: #1D
    def __init__(self,start=40,end=200,node_range=10):
        self.start = start 
        self.end = end
        self.node_range = 10 
        self.grid = []
        self.nodes = SLinkedList()
        self.drones = []
    #getters

    def get_node(self, altitude: int) -> Node: # gets node at altitude 
        for node in self.grid:
            if node.altitude_start<altitude and node.altitude_end>=altitude:
                return node
    def get_lowest_cost_node(self,node:Node): #NEEDS FIXING
        prev_node:Node = node.prevval
        next_node:Node = node.nextval
        if prev_node!=None and next_node!=None:
            if prev_node.get_cost()<node.get_cost():
                return prev_node
            elif next_node.get_cost()>node.get_cost():
                return prev_node
            else:
                print("no movement") # Could implement recursive function, but not necessary since there is only two drones 
                return node
        elif prev_node!=None and next_node==None:
            if(next_node.get_cost()>node.get_cost()):
                return node
            else:
                return next_node
        elif prev_node==None and next_node!=None:
            if(next_node.get_cost()>node.get_cost()):
                return node
            else:
                return next_node
            return next_node # only reasonable cause there is only one drone
    def intialize(self,start=40,end=200): # intialize nodes along altitude
        self.start = start + self.node_range # first node is already done, because of head intialization
        self.end = end 
        self.nodes.headval = Node({'start':start,'end':start+self.node_range}) # intialize head of linked list
        node = self.nodes.headval # intialize first node
        for i in range(self.start,self.end,self.node_range):
            interval = {'start':i,'end':i+self.node_range}
            node.nextval = Node(interval)
            node.nextval.prevval = node
            node = node.nextval
            self.grid.append(node.prevval)
        return self


    def add_obstacle_to_node(self, altitude:int): # fix it to be node as input(ONLY USED FOR TESTING)
        node: Node = self.get_node(altitude)
        node.set_obstacle_on_me(True)
        return self
    def remove_obstacle_from_node(self,altitude:int):
        node: Node = self.get_node(altitude)
        node.set_obstacle_on_me(False)
        return False
    
    # CAN BE WRITTEN BETTER
    def update_obstacles(self,drones: dict[str,Drone]):#Only problem with this method late(1 sec) with the update
        for drone in drones.values(): 
            node_drone:Node = drone.get_node() # drones attached node
            node_alt:Node = self.get_node(drone.get_altitude())  # drones altitude corresponding node 

            if node_drone is None: # drone is not attached to a node
                node_alt.add_drone(drone)
                drone.set_node(node_alt)
                if isinstance(drone,EnemyDrone):
                    drone.get_node().set_cost(50)
            elif node_drone!=node_alt: # drones has moved to a new node
                if isinstance(drone,EnemyDrone):
                    drone.get_node().set_cost(1)
                node_drone.remove_drone(drone)
                node_alt.add_drone(drone)
                drone.set_node(node_alt)
            else: 
                node_alt.add_drone(drone)
                drone.set_node(node_alt)
                if isinstance(drone,EnemyDrone):
                    drone.get_node().set_cost(50)



    def print_grid(self): # print the grid
        for node in self.grid:
            print(node.get_interval(),node.drones_on_me,"cost: ", node.cost)




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
        pdb.set_trace()  # Start debugger here
        # Parse the incoming message
        remote_id_data = json.loads(msg.data)
        drone_id = remote_id_data['drone_id']

        # Add new drone or update existing drone
        if drone_id not in drones:
            drones[drone_id] = EnemyDrone().update_data(remote_id_data)
            self.get_logger().info(f"New Drone Added: {drone_id} ,{drones[drone_id]}")
        grid.update_obstacles(drones)
        drone_1.move_to_node(grid.get_lowest_cost_node())

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = RemoteIDSubscriberNode()
    pdb.set_trace()  # Start debugger here
    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()

#Intial values
altitude_inteval = {'start':50,'end':300}
grid = Grid().intialize() # Find a way to remove obstacle
drone_1 = Drone().set_grid(grid).set_altitude(60)
drone_1.set_id('drone54321') # add to droens 
cycle = 0 


drones['drone54321'] = drone_1
drones['drone12345'] = EnemyDrone().update_data({'drone_id':'drone12345','location':{'latitude':37.7749,'longitude':-122.4194,'altitude':100.5},'velocity':{'x':5.0,'y':0.0,'z':0.0},'status':'OK','timestamp':0})

# One problem
while cycle < 20: # while loop for testing # Issue with the update_obstacles method, it is not removing the 'drone12345' from the node 100,109
    print("CYCLE: ",cycle)
    if cycle ==2:
        drones['drone12345'].altitude = 60
    if cycle ==10:
        drones['drone12345'].altitude = 42
    grid.update_obstacles(drones)
    grid.print_grid()
    lowest_code_node = grid.get_lowest_cost_node(drone_1.get_node())
    print("From altitude: ", drone_1.get_altitude())
    drone_1.move_to_node(lowest_code_node)
    print("To altitude ", drone_1.get_altitude())

    cycle += 1