import numpy as np
from geometry_msgs.msg import Point, Twist, PoseStamped

class R_Astar():
    
    def __init__(start, goal):

        self.goal = goal
        self.start = start


    def  heuristic_manhattan(robot_position,goal_position):
        
        robot_x, robot_y = robot_position
        goal_x, goal_y = goal_position

        m_d = abs(goal_x - robot_x) + abs(goal_y-robot_y) 

        return m_d

    def child_generator(state,robot_position):

        robot_x, robot_y = robot_position
        available_moves = []
        robot_idx_x = state.index[robot_x]
        robot_idx_y = state.index[robot_y]


        for neighbor_x in range(robot_idx_x-1, robot_idx_x+2):

            for neighbor_y in range(robot_idx_y-1, robot__idx_y+2):

                if state[neighbor_x,neighbor_y] == 0:

                    available_moves.append(state)

                    







        
        

        
        return


