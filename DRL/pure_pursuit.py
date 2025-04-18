import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid , Odometry , Path
from geometry_msgs.msg import PoseStamped , Pose, Twist
from sensor_msgs.msg import Imu 
import math
import scipy.interpolate as si
from rclpy.qos import QoSProfile
import threading ,time
import matplotlib.pyplot as plt

from turtlebot3_msgs.msg import Result, Acceleration

import csv

# Episode outcome enumeration
UNKNOWN = 0
SUCCESS = 1

lookahead_distance = 0.2
speed = 0.4
timestart = 0.0

def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


def pure_pursuit(current_x, current_y, current_heading, path,index):
    global lookahead_distance
    closest_point = None
    v = speed
    for i in range(index,len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path)-1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/6
        v = 0.0
    else:
        desired_steering_angle = 0.0
    return v,desired_steering_angle,index


class navigationControl(Node):
    def __init__(self):
        super().__init__('Navigation')
        self.subscription = self.create_subscription(Result,'/episode_result',self.episode_result_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.info_callback,10)
        self.subscription = self.create_subscription(Pose,'goal_pose',self.goal_pose_callback,QoSProfile(depth=10))
        self.subscription = self.create_subscription(Acceleration, '/accel',self.accel_callback, 10)
                                                     
        self.publisher_visual_path = self.create_publisher(Path, '/visual_path', 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.episode_result = UNKNOWN
        self.acceleration = 0
        self.cmd_vel_times = []
        self.cmd_vel_linear = []

        self.accel_times = []
        self.accel_value = []
        self.timestamp = 0.0
        self.stage = 0
        with open('/tmp/drlnav_current_stage.txt', 'r') as f:
            self.stage = int(f.read())
        print(f"planner running on stage: {self.stage}")
        print("Pure Pursuit Starting...")
    
    def episode_result_callback(self,msg):
        self.episode_result = msg.value
    
    def accel_callback(self,msg):
        self.acceleration = msg.value
        self.accel_times.append(time.time() - timestart)
        self.accel_value.append(msg.value)

    def goal_pose_callback(self,msg):
        self.goal = (msg.position.x,msg.position.y)
        myThread = threading.Thread(target=self.follow_path)

        myThread.start()
        self.cmd_vel_times.clear()
        self.cmd_vel_linear.clear()
        
        self.accel_times.clear()
        self.accel_value.clear()

    def follow_path(self):
        global timestart

        timestart =  time.time()
        while True:

            csv_file_path = f"path_data_{self.stage}.csv"
            with open(csv_file_path, mode="r") as file:
                reader = csv.DictReader(file) 
                self.path  = [(float(row["X"]), float(row["Y"])) for row in reader]

            twist = Twist()
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for i in range(len(self.path)):
                pose = PoseStamped()
                pose.pose.position.x = self.path[i][0]
                pose.pose.position.y = self.path[i][1]
                path_msg.poses.append(pose)
            
            self.publisher_visual_path.publish(path_msg)
            v = 0.0
            w = 0.0
            i = 0

            while(self.episode_result != UNKNOWN):
                time.sleep(0.1)

            while True:
                if not hasattr(self, 'x'):
                    continue
                v , w ,i = pure_pursuit(self.x,self.y,self.yaw,self.path,i)

                if(self.episode_result != UNKNOWN):
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    flag = True
                    #print("break restart environment!")
                    break
 
                if(v != 0):
                    twist.linear.x = v + self.acceleration 
                else:
                    twist.linear.x = v
                
                twist.angular.z = w

                if(twist.linear.x > 0.6):
                    twist.linear.x = 0.6
                elif(twist.linear.x < 0.0):
                    twist.linear.x = 0.0
                
                #print("self.acceleration:",self.acceleration)
                self.publisher_visual_path.publish(path_msg)
                self.publisher.publish(twist)

                self.cmd_vel_linear.append(twist.linear.x)
                self.cmd_vel_times.append(time.time() - timestart)
                time.sleep(0.1)
            if(flag == True):
                print("break")
                break


    def info_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)



def main(args=None):
    rclpy.init(args=args)
    navigation_control = navigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
