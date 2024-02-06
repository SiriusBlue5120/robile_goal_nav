import numpy as np
from geometry_msgs.msg import Point, Twist, PoseStamped

class R_Astar():
    
    def __init__(self,start, goal):

        self.goal = goal
        self.start = start


    def  heuristic_manhattan(robot_position,goal_position):
        
        robot_x, robot_y = robot_position
        goal_x, goal_y = goal_position

        m_d = abs(goal_x - robot_x) + abs(goal_y-robot_y) 

        return m_d

    def child_generator(self,state,robot_position):

        '''
        :param state: a current state of the world
        robot_position: our current position of the robot in state
        Returns a tuple of eligible children indexes to explore
        '''

        robot_x, robot_y = robot_position
        available_moves = []

        idx_x = state.index[robot_x]
        idx_y = state.index[robot_y]


        for neighbor_x in range(idx_x-1, idx_x+2):
            for neighbor_y in range(idx_y-1, idx_y+2):

                available_moves.append((neighbor_x,neighbor_y))

        return available_moves
                    




