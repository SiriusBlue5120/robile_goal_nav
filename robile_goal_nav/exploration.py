import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class Exploration(Node):

    def __init__(self, behavior=False):        
        super().__init__(node_name="Exploration")
        # Logging
        self.verbose = False
        self.behavior = behavior

        self.data_grid: np.ndarray
        self.height = 0
        self.width = 0
        self.origin = Pose()
        self.resolution = 0.0
        self.robot_pose = None

        self.radius = 25
        self.random = True

        # Attempts init
        self.attempts = 50

        # Frames
        self.odom_frame = 'map'
        self.robot_frame = 'base_link'

        if not self.behavior:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

            # subscription for map
            self.scan_subscriber = self.create_subscription(
                OccupancyGrid, "/map",
                self.get_occupancy_grid,
                qos_profile=rclpy.qos.qos_profile_sensor_data
                )
            # Subscription to robot pose
            self.pose_subscriber = self.create_subscription(
                PoseWithCovarianceStamped,
                "/pose",
                self.pose_callback,
                10
                )
            # publisher for pose
            self.pose_publisher = self.create_publisher(
                PointStamped, 
                '/pose_explore',
                10
                )     
        else:
            self.scan_subscriber = None
            self.pose_subscriber = None
            self.pose_publisher = None

        self.pose_next = PointStamped()
        
        # Flags
        self.goal_explored = True

        self.validation = True


    def set_goal_explored(self):
        self.goal_explored = True


    def reset_goal_explored(self):
        self.goal_explored = False
        

    def get_occupancy_grid(self, msg:OccupancyGrid):

        # print(self.data_grid)
        self.height = msg.info.height
        self.width = msg.info.width
        self.origin = msg.info.origin
        # print(f"self.origin: {self.origin}")
        self.resolution = msg.info.resolution

        self.data_grid = np.array(msg.data, dtype=np.int64).reshape(\
            (self.height, self.width), order='F')

        # data_in_2d = self.data_grid.reshape((self.height, self.width), order='F')

        # if self.goal_explored or self.validation:
        self.calculate_fringe_pose()
        self.pose_next.header.frame_id = self.odom_frame
        self.pose_next.header.stamp = msg.header.stamp

        if not self.behavior:
            self.pose_publisher.publish(self.pose_next)

        self.reset_goal_explored()


    def pose_callback(self,msg:PoseWithCovarianceStamped) -> None:

        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y

        self.robot_pose = (robot_x,robot_y)

        return None


    def calculate_fringe_pose(self):
            
        condition = False
        point_list = []
        failed_points = []

        attempts = 0
        while attempts < self.attempts:

            # Values for random sampling
            x = np.random.randint(0,self.height)
            y = np.random.randint(0,self.width)
            random_point = (x,y)

            radius = self.radius

            data_slice = self.data_grid[x-radius:x+radius,y-radius:y+radius]
            data_slice_size = np.size(data_slice)

            free_space = np.sum(data_slice == 0) / data_slice_size
            occupied_space = np.sum(data_slice > 0) / data_slice_size
            unknown_space = np.sum(data_slice == -1) / data_slice_size

            # self.get_logger().info(f"free: {free_space} | occupied_space: {occupied_space} | unknown_space: {unknown_space}")

            condition = (unknown_space >= 0.7) and (free_space >= 0.4) and (occupied_space < 0.01)

            attempts += 1

            if condition:
                point_list.append(random_point)
            else:
                failed_points.append(random_point)

        if not point_list:
            point_list = failed_points

        # print(f"point_list: {point_list}")

        point_list = np.array(((point_list)))
        diference_list = point_list - self.robot_pose
        norm_vec = np.linalg.norm(diference_list,axis=1)
        min_value_idx = np.argmin(norm_vec)

        if self.random:
            min_value_idx = np.random.choice(range(point_list.shape[0]), 1)[0]

        point_next = point_list[min_value_idx]

        if self.verbose:
            print('------------')
            print(point_next)

        self.pose_next.point.x = point_next[0]*self.resolution + self.origin.position.x
        self.pose_next.point.y = point_next[1]*self.resolution + self.origin.position.y
        # self.pose_next.pose.position.z = self.random_point[2]*self.resolution + self.origin.position.z



def main(args=None):
    rclpy.init(args=args)

    node= Exploration()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()