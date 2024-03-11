from heapq import heapify, heappop, heappush
import skimage.measure
import numpy as np
import rclpy
from geometry_msgs.msg import (PointStamped, Pose, PoseStamped,
                               PoseWithCovarianceStamped)
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node


class R_Astar(Node):
    
    def __init__(self, behavior=False):
        super().__init__(node_name="A_star")

        self.behavior = behavior

        self.timestamp = {}
        self.robot_pose = np.zeros(2)
        self.goal_pose = np.zeros(2)
        self.state : np.array
        self.compression: int = 8

        self.validation = True
        self.usePose = True

        if not self.behavior:
            # Map subscriber
            self.subscriber_map = self.create_subscription(
                OccupancyGrid,
                "/map",
                self.map_callback,
                10
                )
            # Pose or Odom listener
            self.subscriber_localization = self.create_subscription(
                PoseWithCovarianceStamped if self.usePose else Odometry,
                "/pose" if self.usePose else "/odom",
                self.localization_callback,
                10
                )
            # Goal position listener
            self.subscriber_goal_pose = self.create_subscription(
                PointStamped,
                "/clicked_point" if self.validation else "/pose_explore",
                self.goal_pose_callback,
                10
                )
            # Creating publisher
            self.publisher_path = self.create_publisher(
                Path,
                "/plan",
                10
                )

        self.origin: Pose
        self.map_frame = 'map'

        self.path_msg: Path = None
        self.no_path = False
        

    def map_callback(self, msg:OccupancyGrid):
        # self.get_logger().info(f'I am recieving the map...')

        self.timestamp = { 
            "sec": msg.header.stamp.sec,
            "nanosec":msg.header.stamp.nanosec
        }

        self.map = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution

        # self.get_logger().info(f"Map of width: {self.width}, height: {self.height}, resolution: {self.resolution}")

        # Converting the map to a 2D array
        self.state = np.reshape(self.map, (self.width, self.height), order='F')
        self.origin = msg.info.origin

        # self.get_logger().info(f"Map of max: {self.state.max()}, min: {self.state.min()}")
        # self.get_logger().info(f'Origin is {self.origin}')

        # State compression 
        self.state: np.ndarray = skimage.measure.block_reduce(self.state,(self.compression,self.compression),np.max,cval=0)
        self.resolution *= self.compression
        self.width = self.state.shape[0]
        self.height = self.state.shape[1]

        # Getting index for robot and goal
        self.robot_idx = self.pose_to_idx(self.robot_pose)
        self.goal_idx = self.pose_to_idx(self.goal_pose)
        # self.get_logger().info(f'robot pose and idx are {self.robot_pose},{self.robot_idx}')
        
        # self.get_logger().info(f'goal pose and goal idx are {self.goal_pose},{self.goal_idx}')



        # Calling A star
        path = self.A_star(self.robot_idx,self.goal_idx)

        if not path:
            self.no_path = True
        else:
            self.no_path = False

        self.create_path_msg(path)

        # Publishing A star path
        if not self.behavior:
            self.publisher_path.publish(self.path_msg)


    def localization_callback (self, msg: Odometry | PoseWithCovarianceStamped):
        #self.get_logger().info(f'Recieving robot pose..')
        #self.get_logger().info(f'Robot pose is {msg.pose.pose.position.x},{msg.pose.pose.position.y}')

        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y


    def goal_pose_callback(self,msg:PointStamped):
        #self.get_logger().info(f'Recieving goal pose...')
        #self.get_logger().info(f'Goal pose is ({msg.point.x},{msg.point.y})')

        self.goal_pose[0] = msg.point.x
        self.goal_pose[1] = msg.point.y


    def create_path_msg(self, path):

        self.path_msg = Path()
        pose_list = []

        # self.get_logger().info(f'Path={path}')

        for idx in path:
            position = self.idx_to_pose(idx)
            pose_msg = PoseStamped()

            pose_msg.pose.position.x = position[0]
            pose_msg.pose.position.y = position[1]

            pose_list.append(pose_msg)

        self.path_msg.poses = pose_list
        self.path_msg.header.stamp.sec = self.timestamp["sec"]
        self.path_msg.header.stamp.nanosec = self.timestamp["nanosec"]
        self.path_msg.header.frame_id = self.map_frame


    def heuristic(self,robot_idx,goal_idx):
        '''
        :robot_idx: in the grid 
        :goal_idx: in the grid
        :returns: Returns the Manhattan heuristic estimation
        '''
        
        robot_x, robot_y = robot_idx[0], robot_idx[1]
        goal_x, goal_y = goal_idx[0],goal_idx[1]

        h_n = np.sqrt(((goal_x - robot_x)**2 + (goal_y-robot_y)**2))

        return h_n


    def child_generator(self,robot_idx):

        ''' 
        :robot_idx: current index of robot position in the world
        :return: a list of tuples of eligible children indexes to explore
        '''

        robot_x, robot_y = (robot_idx[0], robot_idx[1])
        available_moves = []

        for neighbor_x in range(robot_x-1, robot_x+2):
            if neighbor_x < 0 or neighbor_x > self.width - 1:
                continue
            
            for neighbor_y in range(robot_y-1, robot_y+2):
                if neighbor_y < 0 or neighbor_y > self.height - 1:
                    continue

                if (neighbor_x,neighbor_y) != (robot_x,robot_y):
                    available_moves.append((neighbor_x,neighbor_y))

        return available_moves
                    

    def pose_to_idx(self, pose):
        '''
        :pose: the pose of robot in the real world
        :return: the idx of the robot and the goal in the grid map 
        '''
        pose_x, pose_y = (pose[0] , pose[1])
        pose_idx_x = round((pose_x - self.origin.position.x) / self.resolution)
        pose_idx_y = round((pose_y - self.origin.position.y) / self.resolution)

        pose_idx_x = min(pose_idx_x, self.width)
        pose_idx_y = min(pose_idx_y, self.height)

        pose_idx_x = max(0, pose_idx_x)
        pose_idx_y = max(0, pose_idx_y)

        return np.abs(np.array((pose_idx_x, pose_idx_y)))
    

    def idx_to_pose(self, robot_idx):
        '''
        :robot_idx: the index of robot in grid map
        :return: the position of robot in real world
        '''
        robot_x, robot_y = (robot_idx[0],robot_idx[1])
        position_x = (robot_x * self.resolution) + self.origin.position.x
        position_y = (robot_y * self.resolution) + self.origin.position.y

        return np.array((position_x, position_y))
        
    
    # def A_star(self,robot_idx,goal_idx):
    #     '''
    #     :robot_idx: the index of robot in grid map
    #     :goal_idx: the index of goal in grid map
    #     :state: a 2D array of current state of the world
    #     '''
    #     robot_idx = tuple(robot_idx)
    #     goal_idx = tuple(goal_idx)

    #     step_cost = 1

    #     fringe = []
    #     expanded_nodes = 0

    #     g_cost = {robot_idx: 0}
    #     h_cost = {robot_idx: self.heuristic(robot_idx, goal_idx)}

    #     explored = []
    #     # explored.append(robot_idx)   # idx of the explored nodes
    #     heapify(fringe)
    #     heappush(fringe,(self.heuristic(robot_idx,goal_idx),robot_idx))


    #     while fringe:
    #         #(len(fringe)>0)
    #         # self.get_logger().info(f'working = {len(fringe)}')
    #         _,current_node = heappop(fringe)
            
    #         # self.get_logger().info(f'current node, goal_idx = {current_node},{type(current_node)}')
    #         # self.get_logger().info(f'goal node, goal_idx = {goal_idx},{type(goal_idx)}')
    #         self.get_logger().info( f'{current_node},{goal_idx},{np.equal(current_node, goal_idx).all()}')

    #         if np.equal(current_node, goal_idx).all():
    #             self.get_logger().info(f'current node, goal_idx = {current_node},{goal_idx}')
    #             return explored
            
    #         else:
    #             explored.append(current_node)
    #             children_idx_list = self.child_generator(current_node)
                
    #             # self.get_logger().info(f'Expanding children: {children_idx_list}')

    #             #self.get_logger().info(f'child generator = {children_idx_list}')
    #             for child_idx in children_idx_list:
    #                 # self.get_logger().info(f'Child idx is {child_idx}')
    #                 if child_idx not in explored:

    #                     child_x,child_y = child_idx

    #                     # self.get_logger().info(f"map state: {self.state[child_y][child_x]}")

    #                     if self.state[child_y][child_x] == 100 or self.state[child_y][child_x] == 1: # Just considering unoccupied cells
    #                         # self.get_logger().info(f'child idx is {child_idx}')
    #                         continue

    #                     estimated_cost = self.heuristic(child_idx,goal_idx)
    #                     total_cost = len(explored)  + estimated_cost
    #                     heappush(fringe,(total_cost,child_idx))  

    #     return explored    

    def A_star(self, robot_idx, goal_idx):
        '''
        :robot_idx: the index of robot in grid map
        :goal_idx: the index of goal in grid map
        :state: a 2D array of current state of the world
        '''
        robot_idx = tuple(robot_idx)
        goal_idx = tuple(goal_idx)

        step_cost = 1

        expanded_nodes = 0

        g_cost = {robot_idx: 0}
        h_cost = {robot_idx: self.heuristic(robot_idx, goal_idx)}

        explored_through = {robot_idx: 0}
        # explored.append(robot_idx)   # idx of the explored nodes

        fringe = []
        heapify(fringe)
        heappush(fringe, (g_cost[robot_idx] + h_cost[robot_idx], robot_idx))

        while fringe:
            #(len(fringe)>0)
            # self.get_logger().info(f'working = {len(fringe)}')
            _, current_node = heappop(fringe)
            expanded_nodes += 1

            # print(f"current_node: {current_node} | len(fringe): {len(fringe)}")

            # self.get_logger().info(f'current node, goal_idx = {current_node},{type(current_node)}')
            # self.get_logger().info(f'goal node, goal_idx = {goal_idx},{type(goal_idx)}')
            # self.get_logger().info( f'{current_node},{goal_idx},{np.equal(current_node, goal_idx).all()}')

            if np.equal(current_node, goal_idx).all():
                # self.get_logger().info(f'current node, goal_idx = {current_node},{goal_idx}')
                path = []

                last_node = current_node
                path.append(last_node)

                while explored_through[last_node]:
                    last_node = explored_through[last_node]

                    path.append(last_node)

                return path[::-1]
            
            else:
                # explored.append(current_node)
            
                children_idx_list = self.child_generator(current_node)                
                for child_idx in children_idx_list:


                    # If obstacle skip
                    child_x, child_y = child_idx
                    if self.state[child_x][child_y] == 100 or self.state[child_x][child_y] == 1:
                        continue

                    # g_estimated_cost = g_cost[current_node] + step_cost
                    # This is sketchy, refactor this
                    g_estimated_cost = g_cost[current_node] + \
                        np.linalg.norm(np.array(current_node) - np.array(child_idx))
                
                    # h_estimated_cost = g_cost[child_idx] + heuristic(child_idx, goal_idx)

                    if child_idx not in g_cost.keys():
                        g_cost[child_idx] = g_estimated_cost

                    # self.get_logger().info(f'Child idx is {child_idx}')
                    if g_estimated_cost <= g_cost[child_idx]:
                        explored_through[child_idx] = current_node
                        g_cost[child_idx] = g_estimated_cost
                        h_cost[child_idx] = g_estimated_cost + self.heuristic(child_idx, goal_idx)

                        total_cost = g_cost[child_idx] + h_cost[child_idx]
                        child_node = (total_cost, child_idx)

                        if child_node not in fringe:
                            heappush(fringe, (total_cost, child_idx))

        return [] 


def main(args=None):
    rclpy.init(args=args)

    node= R_Astar()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()

