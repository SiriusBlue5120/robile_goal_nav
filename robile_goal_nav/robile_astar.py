import numpy as np
from typing import Callable, List, Tuple
from geometry_msgs.msg import Point, Twist, PoseStamped
from heapq import heappop,heappush,heapify


class R_Astar():
    
    def __init__(self,start, goal):

        self.goal = goal
        self.start = start
        self.resolution = 0.1
        self.map_size = (20,20)


    def heuristic(self,robot_position,goal_position):
        '''
        :robot_position: in the real world
        :goal_position: in the real world
        :returns: Returns the Manhattan heuristic esrimation
        '''
        
        robot_x, robot_y = robot_position
        goal_x, goal_y = goal_position

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
                    

    def pose_to_idx (self,robot_position,goal_position):
            '''
            :robot_position: the position of robot in the real world
            :goal_position: The goal position in the real world
            :return: the idx of the robot and the goal in the grid map 
            '''
            
            cell_counts = tuple(int(dim / self.resolution + ((dim / self.resolution) + 1) % 2) \
                for dim in self.map_size)
            self.grid_center = tuple(int((dim - 1) / 2) for dim in cell_counts)

            robot_x,robot_y = robot_position
            robot_grid_x = round(robot_x/self.resolution)+ self.grid_center[0]
            robot_grid_y = round(robot_y/self.resolution)+ self.grid_center[1]

            goal_x,goal_y = goal_position
            goal_grid_x = round(goal_x/self.resolution)+ self.grid_center[0]
            goal_grid_y = round(goal_y/self.resolution)+ self.grid_center[1]

            return ((robot_grid_x,robot_grid_y),(goal_grid_x,goal_grid_y))
    

    def idx_to_pose(self,robot_idx):
        '''
        :robot_idx: the index of robot in grid map
        :return: the posiition of robot in real world
        '''
        robot_x,robot_y = robot_idx
        position_x = (robot_x - self.grid_center) * self.resolution 
        position_y = (robot_y - self.grid_center) * self.resolution 
        return ((position_x,position_y))




    def A_star(self,robot_position, goal_position, robot_idx, goal_idx,state):
        '''
        :robot_position: the position of robot in the real world
        :goal_position: The goal position in the real world
        :robot_idx: the index of robot in grid map
        :goal_idx: the index of goal in grid map
        :state: the current state of the world
        '''
        explored = set()   # idx of the explored nodes
        explored.add(robot_idx)
        fringe = []
        heapify(fringe)
        heappush(fringe,(self.heuristic(robot_position,goal_position),robot_idx))


        while fringe:
            
            _,current_node = heappop(fringe)

            if current_node == goal_idx:
                return explored
            
            else:
                children_idx_list = self.child_generator(current_node)

                for child_idx in children_idx_list:
                    
                    if child_idx not in explored:

                        child_x,child_y = child_idx

                        if state[child_x][child_y] != 1: # Just considering unoccupied cells

                            explored.add(child_idx)
                            distance = self.heuristic(self.idx_to_pose(child_idx),goal_position)
                            heappush(fringe,(distance,child_idx))  


        return explored





