#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import math
from std_msgs.msg import Int32
from scipy.special import fresnel
from std_srvs.srv import Empty

class drawcycle (Node):

    #constructor
    def __init__(self):
        super().__init__("cycle") 
        self.publisher_= self.create_publisher(Twist,"/turtle1/cmd_vel", 10)
        self.current_index = 0 
        self.K_linear = 3.0
        self.K_angular = 3.0
        self.current_x = 5.5
        self.current_y = 5.5
        self.turtle_pose = Pose()
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.subscriber = self.create_subscription(Int32, 'shape', self.listener_callback, 10)

        self.path_points = []
        self.timer=self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("draw started")

    def listener_callback(self, msg):
        self.reset_turtle()  
        self.current_index = 0  
        if msg.data == 1:
            self.path_points = self.path_cornu()
        elif msg.data == 2:
            self.path_points = self.path_deltoid()
        elif msg.data == 3:
            self.path_points = self.path_spiral()
        elif msg.data == 4 :
            self.path_points = self.path_astroid()
        elif msg.data == 5:
            self.path_points = self.path_flower()

    def pose_callback(self, msg: Pose):
        self.turtle_pose = msg

    from std_srvs.srv import Empty

    def reset_turtle(self):
        client = self.create_client(Empty, "/reset")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /reset service...")
        request = Empty.Request()
        client.call_async(request)

    
    def path_deltoid(self):

        path_points =[]
        for t in np.linspace(0, 2*math.pi, 200):  
            scale= 0.1
            x = scale * (2*math.cos(t) + math.cos(2*t))   
            y = scale * (2*math.sin(t) - math.sin(2*t))
            
            dx = -2*math.sin(t) - 2*math.sin(2*t)
            dy =  2*math.cos(t) - 2*math.cos(2*t)
            theta = math.atan2(dy, dx)
            path_points.append([x,y,theta])
        return path_points

    
    def path_cornu(self):
        path_points = []
        scale = 0.5

        t = np.linspace(-2, 2, 200)
        S, C = fresnel(t)

        x = scale * C
        y= scale * S

        for i in range(1, len(t)):
            dx = x[i] - x[i-1]
            dy = y[i] - y[i-1]
            theta = math.atan2(dy, dx)
            path_points.append([x[i], y[i], theta])
        return path_points

    def path_spiral(self):
        path_points = []
        a = 0.1    
        b = 0.15   
        scale = 0.5
        for t in np.linspace(0, 6*math.pi, 200):  
            r = a * math.exp(b * t)   
            x = scale*(r * math.cos(t))
            y = scale*(r * math.sin(t))

            dx = (a * math.exp(b*t)) * (b*math.cos(t) - math.sin(t))
            dy = (a * math.exp(b*t)) * (b*math.sin(t) + math.cos(t))
            theta = math.atan2(dy, dx)

            path_points.append([x, y, theta])
        return path_points

    def path_astroid(self):
        path_points = []
        a = 1.0
        scale = 0.4
        for t in np.linspace(0, 2*math.pi, 200):   
            x = scale*(a*((math.cos(t))**3))
            y = scale*(a*((math.sin(t))**3))

            dx = -3*a*(math.cos(t)**2)*math.sin(t)
            dy =  3*a*(math.sin(t)**2)*math.cos(t)
            theta = math.atan2(dy, dx)

            path_points.append([x, y, theta])
        return path_points

    def timer_callback(self):
       
        if self.current_index >= len(self.path_points) - 1:
            self.get_logger().info("Finished path")
            return

        x, y, theta = self.path_points[self.current_index]
        x_next, y_next, _ = self.path_points[self.current_index + 1]

        dx = x_next - x
        dy = y_next - y
        dt = 0.1  # matches timer period

        v = math.sqrt(dx**2 + dy**2) / dt
        heading = math.atan2(dy, dx)
        angle_diff = heading - self.turtle_pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = 2.0 * angle_diff
        self.publisher_.publish(msg)

        self.current_index += 1

    


def main (args=None):
    rclpy.init(args=args)
    node=drawcycle()
    rclpy.spin(node)
    rclpy.shutdown()



