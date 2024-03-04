
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import  Pose, PointStamped
from geometry_msgs.msg import TransformStamped
import tf_transformations as tf
import tf2_ros

class Exploration(Node):

    def __init__(self,args=None):        
        super().__init__(node_name="Exploration")
        # Logging
        self.verbose = True

        # subscription for map
        self.scan_subscriber = self.create_subscription(
                OccupancyGrid, "/map",
                self.get_occupancy_grid,
                10)
        self.data_grid: np.ndarray
        self.height = 0
        self.width = 0
        self.origin = Pose()
        self.resolution = 0.0
        self.random_point = np.zeros(2, dtype=np.int64)
        self.odom_frame = 'map'
        self.robot_frame = 'base_link'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # publisher for pose
        self.pose_next = PointStamped()
        self.pose_publisher = self.create_publisher(
            PointStamped, '/pose_explore',
            10)
        

    def get_occupancy_grid(self, msg:OccupancyGrid):

        self.data_grid = np.array(msg.data, dtype=np.int64)
        # print(self.data_grid)
        self.height = msg.info.height
        self.width = msg.info.width
        self.origin = msg.info.origin
        # print(f"self.origin: {self.origin}")
        self.resolution = msg.info.resolution
        self.calculate_fringe_pose()
        self.pose_next.header.frame_id = self.odom_frame
        self.pose_next.header.stamp = msg.header.stamp
        self.pose_publisher.publish(self.pose_next)
        

    def calculate_fringe_pose(self):
        data_in_2d = self.data_grid.reshape((self.height, self.width), order='F')
            
        # Values to random sampling
        self.random_point = np.zeros(2, dtype=np.int64)
        condition = False

        while not condition:
            x = np.random.randint(0,self.height)
            y = np.random.randint(0,self.width)
            radius = 500

            data_slice = data_in_2d[x-radius:x+radius,y-radius:y+radius]
            data_slice_size = np.size(data_slice)

            free_space = (np.sum(data_slice == 0, dtype=np.float32) / data_slice_size)
            occupied_space = (np.sum(data_slice > 0, dtype=np.float32) / data_slice_size)
            unknown_space = (np.sum(data_slice == -1, dtype=np.float32) / data_slice_size)
            condition = (0.1 <= free_space <= 0.3) and (0.5 <= unknown_space <= 1.0)
                    
            # TODO: figure the condition
            if condition :
                self.random_point[0:2] = [x, y]

                break

        if self.verbose:
            print('------------')
            print(self.random_point)

        self.pose_next.point.x = self.random_point[0]*self.resolution + self.origin.position.x
        self.pose_next.point.y = self.random_point[1]*self.resolution + self.origin.position.y
        # self.pose_next.pose.position.z = self.random_point[2]*self.resolution + self.origin.position.z



def main(args=None):
    rclpy.init(args=args)

    node= Exploration()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()