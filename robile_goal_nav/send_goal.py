import rclpy
from rclpy.node import Node
# importa el "tipo de mensaje" 
# 1)ros2 interface list |grep String;ros2 interface show std_msgs/msg/String
from geometry_msgs.msg import Twist, PoseStamped 
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs
import numpy as np

class SendGoal(Node):
    def __init__(self,args=None):        
        super().__init__(node_name="send_goal")
        # Logging
        self.verbose = True

        # Setting up buffer and transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Set frames
        self.source_frame = 'base_link'
        self.target_frame = 'odom'

        # Twist command
        self.vel_command = Twist()

        # Velocity limits
        self.max_angular_vel = 0.75
        self.max_linear_vel = 0.75

        self.angle_threshold = 0.05
        self.distance_threshold = 0.25

        # Set input goal wrt base_link
        assert len(args) == 3
        self.set_goal([args[0], args[1],     0.0], 
                      [    0.0,     0.0, args[2]],
                      self.target_frame)
        
        # Creacion de publisher
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # State definitions
        self.IDLE = 0
        self.INIT = 1
        self.LOOK_AT_GOAL = 2
        self.TRAVEL_TO_GOAL = 3
        self.ALIGN_AT_GOAL = 4

        # State
        self.state = self.INIT
        

    def set_goal(self, position, orientation_euler, frame_id):
        '''
        Set desired goal pose wrt a frame
        '''
        self.goal = PoseStamped()

        self.goal.header.frame_id = frame_id

        self.goal.pose.position.x = position[0]
        self.goal.pose.position.y = position[1]
        self.goal.pose.position.z = position[2]

        goal_orientation = quaternion_from_euler(*orientation_euler)

        self.goal.pose.orientation.x = goal_orientation[0]
        self.goal.pose.orientation.y = goal_orientation[1]
        self.goal.pose.orientation.z = goal_orientation[2]
        self.goal.pose.orientation.w = goal_orientation[3]

        if self.verbose:
            self.get_logger().info(f"goal set: {self.goal}")


    def calculate_transform(self, target_frame, source_frame):
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        
        if self.verbose and False:
            self.get_logger().info(f"transform btw target {target_frame} and " + \
                                   f"source {source_frame}: {transform}")

        return transform


    def compute_robot_heading(self):
        orientation_ypr = euler_from_quaternion([
                self.pose.transform.rotation.x, self.pose.transform.rotation.y,
                self.pose.transform.rotation.z, self.pose.transform.rotation.w
            ])
        
        return orientation_ypr[2]
    

    def compute_translation_to_goal(self):
        translation_to_goal = [
            self.goal.pose.position.x - self.pose.transform.translation.x,
            self.goal.pose.position.y - self.pose.transform.translation.y,
            self.goal.pose.position.z - self.pose.transform.translation.z,
            ]
        
        return translation_to_goal
    

    def compute_heading_to_goal(self):          
        translation_to_goal = self.compute_translation_to_goal()

        heading_to_goal = np.arctan2(translation_to_goal[1], \
                                    translation_to_goal[0])
        
        return heading_to_goal


    def compute_distance_to_goal(self):
        translation_to_goal = self.compute_translation_to_goal()

        distance_to_goal = np.linalg.norm(translation_to_goal)

        return distance_to_goal


    def control_loop(self):
        # Robot pose
        self.pose = self.calculate_transform(self.target_frame, self.source_frame)

        if self.verbose:
            self.get_logger().info(f"current state: {self.state}")
            self.get_logger().info(f"current {self.source_frame} pose wrt " + 
                                   f"{self.target_frame}: {self.pose}")
            

        if self.state == self.INIT:
            # Transforming goal to target frame if in source frame
            if self.goal.header.frame_id == self.source_frame:
                self.goal = tf2_geometry_msgs.do_transform_pose_stamped(self.goal, self.pose)
                self.goal.header.frame_id = self.target_frame

                if self.verbose:
                    self.get_logger().info(f"goal transformed to target {self.target_frame}")
                    self.get_logger().info(f"transformed pose: {self.goal}") 

            # Init complete, look at goal next
            self.state = self.LOOK_AT_GOAL


        if self.state == self.LOOK_AT_GOAL:
            robot_heading = self.compute_robot_heading()
            
            heading_to_goal = self.compute_heading_to_goal()
            
            if self.verbose:
                self.get_logger().info(f"robot_heading: {robot_heading}")
                self.get_logger().info(f"heading_to_goal: {heading_to_goal}")  

            heading_error = heading_to_goal - robot_heading
            if np.abs(heading_error) > self.angle_threshold:
                self.vel_command.angular.z = self.max_angular_vel * \
                                        np.sign(heading_error)
            else:
                self.vel_command.angular.z = 0.0
                
                # Heading achieved, travel to goal next
                self.state = self.TRAVEL_TO_GOAL


        if self.state == self.TRAVEL_TO_GOAL:
            robot_heading = self.compute_robot_heading()
            distance_to_goal = self.compute_distance_to_goal()
            heading_to_goal = self.compute_heading_to_goal()
            heading_error = heading_to_goal - robot_heading

            if self.verbose:
                self.get_logger().info(f"distance_to_goal: {distance_to_goal}")
                self.get_logger().info(f"robot_heading: {robot_heading}")
                self.get_logger().info(f"heading_to_goal: {heading_to_goal}")  

            if distance_to_goal > self.distance_threshold:
                if np.abs(heading_error) > self.angle_threshold:
                    self.vel_command.angular.z = self.max_angular_vel * \
                                        np.sign(heading_error) / 5
                else:
                    self.vel_command.angular.z = 0.0

                self.vel_command.linear.x = self.max_linear_vel 
                self.vel_command.linear.y = 0.0 


            else:
                self.vel_command.linear.x = 0.0
                self.vel_command.linear.y = 0.0
                self.vel_command.angular.z = 0.0

                # Position achieved, align at goal next
                self.state = self.ALIGN_AT_GOAL


        if self.state == self.ALIGN_AT_GOAL:
            robot_heading = self.compute_robot_heading()

            goal_heading = euler_from_quaternion([
                self.goal.pose.orientation.x,
                self.goal.pose.orientation.y,
                self.goal.pose.orientation.z,
                self.goal.pose.orientation.w,
            ])[2]
            
            heading_error = goal_heading - robot_heading

            if self.verbose:
                self.get_logger().info(f"robot_heading: {robot_heading}")
                self.get_logger().info(f"goal_heading: {goal_heading}")  

            if np.abs(heading_error) > self.angle_threshold:
                self.vel_command.angular.z = self.max_angular_vel * \
                                        np.sign(heading_error)
            else:
                self.vel_command.angular.z = 0.0

                self.state = self.IDLE


        # Publish vel_command
        if self.verbose:
            self.get_logger().info(f"vel_command: {self.vel_command}")

        self.publisher.publish(self.vel_command)
            


def main(args=None)-> None:
    rclpy.init(args=args)

    pose_robile = [float(x) for x in input('Enter a pose as (x, y, theta):').split(',')]
    node = SendGoal(pose_robile)

    rclpy.spin(node)
    rclpy.shutdown()