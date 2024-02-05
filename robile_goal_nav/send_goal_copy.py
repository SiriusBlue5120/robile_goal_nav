import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseStamped               # importa el "tipo de mensaje" 1)ros2 interface list |grep String;ros2 interface show std_msgs/msg/String
import tf2_ros
from tf_transformations import euler_from_quaternion
import math
import numpy as np


class SendGoal(Node):
    def __init__(self,args=None):
        if args is None:
            args = [2.0, -3.0 -2.0]
        
        super().__init__(node_name="send_goal")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # save the input goal location with respect to base_laser
        self.goal_wrt_base_link = PoseStamped()
        self.goal_wrt_base_link.pose.position.x = args[0]
        self.goal_wrt_base_link.pose.position.y = args[1]
        self.orientation_wrt_odom = args[2]
        self.theta_wrt_odom = math.atan(args[1]/args[0])
        self.target_frame='odom'
        self.source_frame ='base_link'
        self.orientation_found = False
        
        self.my_publisher = self.create_publisher(Twist,"/cmd_vel",10)        #creacion de publisher
        timer = self.create_timer(0.5, self.my_callback)

    def my_callback(self)->None:

        my_msg = Twist()
        self.calculate_orientation(self.target_frame,self.source_frame,self.theta_wrt_odom)
        if self.orientation_found and not math.isclose(self.calculate_transform(self.target_frame,self.source_frame).transform.translation.x,self.goal_wrt_base_link.pose.position.x):
            my_msg.linear.x = 0.0
            my_msg.linear.y = 0.0
            my_msg.linear.z = 0.0
            my_msg.angular.x = 0.0
            my_msg.angular.y = 0.0
            my_msg.angular.z = 0.5
            print(self.calculate_transform(self.target_frame,self.source_frame).transform.translation.x)

        else:
        
            distance = self.calculate_distance('odom', 'base_link')
            print("Distance to goal: ", distance)
            my_msg.linear.x = 1.0
            my_msg.linear.y = 0.0
            my_msg.linear.z = 0.0
            my_msg.angular.x = 0.0
            my_msg.angular.y = 0.0
            my_msg.angular.z = 0.0
          
            if np.isclose(distance,0):
                my_msg.linear.x = 0.0
                my_msg.linear.y = 0.0
                my_msg.linear.z = 0.0
                self.target_frame='base_link'
                self.source_frame ='odom'
                self.theta_wrt_odom = self.orientation_wrt_odom
                self.orientation_found = False

        self.my_publisher.publish(my_msg)
       
    def calculate_distance(self,target_frame,source_frame) -> int:
        transform = self.calculate_transform(target_frame, source_frame)
        print("base_link wrt odom: ", transform.translation.x, transform.translation.y)
        distance = math.sqrt((transform.translation.x-self.goal_wrt_base_link.pose.position.x)**2
                                  + (transform.translation.y-self.goal_wrt_base_link.pose.position.y)**2)
        return distance
    
    def calculate_orientation(self, odom_frame,base_link_frame,angle):

        # For better intuition, print different transformations between different frame
        current_angle=0
        if not self.orientation_found:   
            transform_link_wrt_odom = self.calculate_transform(base_link_frame,odom_frame)
            print("\ntransform_link_wrt_odom\n", transform_link_wrt_odom)
            quaternion = [transform_link_wrt_odom.transform.rotation.x,transform_link_wrt_odom.transform.rotation.y,transform_link_wrt_odom.transform.rotation.z,transform_link_wrt_odom.transform.rotation.w]
            euler_angle = euler_from_quaternion(quaternion)
            current_angle = euler_angle[0] - angle
            print(current_angle,euler_angle[0])
            self.orientation_found = True
        
        return current_angle    


    def calculate_transform(self,ini_frame,final_frame):
        return self.tf_buffer.lookup_transform(ini_frame,final_frame,rclpy.time.Time())




def main(args=None)-> None:
    rclpy.init(args=args)
    pose_robile = [float(x) for x in input('Enter a pose(x,y,theta):').split(',')]
    node= SendGoal(pose_robile)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()