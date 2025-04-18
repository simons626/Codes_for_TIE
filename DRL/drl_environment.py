#!/usr/bin/env python3

import math
import numpy
import sys
import copy
from numpy.core.numeric import Infinity
import csv

import time

from geometry_msgs.msg import Pose, Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from turtlebot3_msgs.srv import DrlStep, Goal, RingGoal
from turtlebot3_msgs.msg import Result, Acceleration, Obstacle

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from . import reward as rw
from ..common import utilities as util
from ..common.settings import ENABLE_BACKWARD, EPISODE_TIMEOUT_SECONDS, ENABLE_MOTOR_NOISE, UNKNOWN, SUCCESS, COLLISION_WALL, COLLISION_OBSTACLE, TIMEOUT, TUMBLE, \
                                TOPIC_SCAN, TOPIC_VELO, TOPIC_ODOM, ARENA_LENGTH, ARENA_WIDTH, MAX_NUMBER_OBSTACLES, OBSTACLE_RADIUS, LIDAR_DISTANCE_CAP, \
                                    SPEED_LINEAR_MAX, SPEED_ANGULAR_MAX, THRESHOLD_COLLISION, THREHSOLD_GOAL, ENABLE_DYNAMIC_GOALS, SPEED_ACCEL_MAX, \
                              MAX_OBSERVE_DISTANCE, MAX_OBSERVE_ANGLE,MAX_REACT_DISTANCE, MAX_DRL_OBSTACLES


"""
...
"""

    def get_state(self, action_previous):

        state = copy.deepcopy(self.scan_ranges)   

        state.append(float(numpy.clip((self.goal_distance / MAX_GOAL_DISTANCE), 0, 1)))     
                                     
        state.append(float(action_previous))

        # dynamic obstacles states:  
            # <1> obstacles distances
            # <2> obstacles angles
            # <3> obstacles linervels                                                                                       

        for obs_distance in state_obs_distances:        #obstacle_distances
            state.append(float(obs_distance))

        for obs_liner in state_obs_linervel:            #obstacle_linervel
            state.append(float(obs_liner))

        for obs_angle in state_obs_angles:              #obstacle_angles
            state.append(float(obs_angle) / math.pi)

        self.local_step += 1

        if self.local_step <= 30: # Grace period to wait for simulation reset
            return state
        # Success
        if self.goal_distance < THREHSOLD_GOAL:
            self.succeed = SUCCESS
        # Collision
        elif self.obstacle_distance1 < THRESHOLD_COLLISION:
            dynamic_collision = False
            for obstacle_distance in self.obstacle_distances:
                if obstacle_distance < (THRESHOLD_COLLISION + OBSTACLE_RADIUS + 0.05):
                    dynamic_collision = True
            if dynamic_collision:
                self.succeed = COLLISION_OBSTACLE
            else:
                self.succeed = COLLISION_WALL
        # Timeout
        elif self.time_sec >= self.episode_deadline:
            self.succeed = TIMEOUT

        if self.succeed is not UNKNOWN:
            self.stop_reset_robot(self.succeed == SUCCESS)

        # Publish episode_result
        episode_result = Result()
        episode_result.value = self.succeed
        self.cmd_result_pub.publish(episode_result)

        return state

"""
...
"""


def main(args=sys.argv[1:]):

    rclpy.init(args=args)
    if len(args) == 0:
        drl_environment = DRLEnvironment()
    else:
        rclpy.shutdown()
        quit("ERROR: wrong number of arguments!")
    rclpy.spin(drl_environment)
    drl_environment.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
