import numpy as np
from typing import Callable, List, Tuple
from geometry_msgs.msg import Point, Twist, PoseStamped,PoseArray,Pose
from heapq import heappop,heappush,heapify
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class R_Astar(Node):
    
    def __init__(self,start):

        self.goal = (26,50)
        self.start = start
        self.resolution = 0.1
        self.map_size = (20,20)
        super().__init__(node_name="A_star")
        self.timestamp = {}
        self.state : np.array

        # Creating subscriber
        self.subscriber = self.create_subscription(
                        OccupancyGrid,
                        "/map",
                        self.listener_callback,
                        10
                        )
        self.origin: Pose
             
        # Creating publisher
        self.publisher = self.create_publisher(
                        PoseArray,
                        "/plan",
                        10
                        )

    def listener_callback(self, msg:OccupancyGrid):
        self.get_logger().info(f'I am recieving the map...')

        self.timestamp = { 
            "sec": msg.header.stamp.sec,
            "nanosec":msg.header.stamp.nanosec
        }

        self.map = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution

        # Converting the map to a 2D array
        self.state = np.reshape(self.map, (self.width,self.height), order='F')
        self.origin = msg.info.origin


    def publish_path(self,path):

        path_msg = PoseArray()
        pose_list = []

        for idx in path:
            position = self.idx_to_pose(idx)
            pose_msg = Pose()

            pose_msg.position.x = position[0]
            pose_msg.position.y = position[1]

            pose_list.append(pose_msg)

        path_msg.poses = pose_list
        path_msg.header.stamp.sec = self.timestamp["sec"]
        path_msg.header.stamp.nanosec = self.timestamp["nanosec"]
        path_msg.header.frame_id = '/map'

        self.publisher.publish(path_msg)
        

    def heuristic(self,robot_idx,goal_idx):
        '''
        :robot_idx: in the grid 
        :goal_idx: in the grid
        :returns: Returns the Manhattan heuristic estimation
        '''
        
        robot_x, robot_y = robot_idx
        goal_x, goal_y = goal_idx

        h_n = abs(goal_x - robot_x) + abs(goal_y-robot_y) 

        return h_n


    def child_generator(self,robot_idx):

        ''' 
        :param robot_position: current index of robot position in the world
        :return: a list of tuples of eligible children indexes to explore
        '''

        robot_x, robot_y = robot_idx
        available_moves = []

        for neighbor_x in range(robot_x-1, robot_x+2):
            for neighbor_y in range(robot_y-1, robot_y+2):

                if (neighbor_x,neighbor_y) != (robot_x,robot_y):

                    available_moves.append((neighbor_x,neighbor_y))

        return available_moves
                    

    def pose_to_idx(self, pose):
            '''
            :pose: the pose of robot in the real world
            :return: the idx of the robot and the goal in the grid map 
            '''
            robot_x, robot_y = pose
            robot_grid_x = round((robot_x - self.origin.position.x) / self.resolution)
            robot_grid_y = round((robot_y - self.origin.position.y) / self.resolution)

            return (robot_grid_x, robot_grid_y)
    

    def idx_to_pose(self, robot_idx):
        '''
        :robot_idx: the index of robot in grid map
        :return: the position of robot in real world
        '''
        robot_x, robot_y = robot_idx
        position_x = (robot_x * self.resolution) + self.origin.position.x
        position_y = (robot_y * self.resolution) + self.origin.position.y

        return (position_x, position_y)


    def A_star(self, robot_idx, goal_idx):
        '''
        :robot_position: the position of robot in the real world
        :goal_position: The goal position in the real world
        :robot_idx: the index of robot in grid map
        :goal_idx: the index of goal in grid map
        :state: a 2D array of current state of the world
        '''
        explored = set(robot_idx)   # idx of the explored nodes
        fringe = []
        heapify(fringe)
        heappush(fringe,(self.heuristic(robot_idx,goal_idx),robot_idx))


        while fringe:
            
            _,current_node = heappop(fringe)

            if current_node == goal_idx:
                return explored
            
            else:
                children_idx_list = self.child_generator(current_node)

                for child_idx in children_idx_list:
                    
                    if child_idx not in explored:

                        child_x,child_y = child_idx

                        if self.state[child_x][child_y] != 1: # Just considering unoccupied cells

                            explored.add(child_idx)
                            estimated_cost = self.heuristic(child_idx,goal_idx)
                            total_cost = len(explored)  + estimated_cost
                            heappush(fringe,(total_cost,child_idx))  


        return explored





