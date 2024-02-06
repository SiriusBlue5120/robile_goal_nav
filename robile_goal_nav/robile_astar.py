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

    def child_generator(self,robot_position):

        '''
        :param 
        robot_position: current index of robot position in the world
        Returns a list of tuples of eligible children indexes to explore
        '''

        robot_x, robot_y = robot_position
        available_moves = []

        for neighbor_x in range(robot_x-1, robot_x+2):
            for neighbor_y in range(robot_y-1, robot_y+2):

                available_moves.append((neighbor_x,neighbor_y))

        return available_moves
                    




