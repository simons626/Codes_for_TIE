from ..common.settings import REWARD_FUNCTION, COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE, SUCCESS, TIMEOUT, RESULTS_NUM, THREHSOLD_ACCEL
import math

goal_dist_initial = 0
accel_count = 0

reward_function_internal = None

def get_reward(succeed, action_accel, min_obstacle_distance, passed_time_distance, cost_time):
    return reward_function_internal(succeed, action_accel, min_obstacle_distance, passed_time_distance, cost_time)

def get_reward_B(succeed, action_accel, min_obstacle_dist, passed_time_distance, cost_time):
        
        Accel_Count_Th = 6 * cost_time
        r_goal = 5 * passed_time_distance

        global accel_count
        if(abs(action_accel) > THREHSOLD_ACCEL):
            accel_count += 1

        if min_obstacle_dist < 0.0:
            r_action = -10.0 * action_accel
        else:
            r_action = 5.0 * action_accel
 
        reward = r_action  + r_goal
 
        if succeed == SUCCESS:
            reward += 2500
            if(accel_count > Accel_Count_Th):
                reward -= 50
            accel_count = 0
 
        elif succeed == COLLISION_OBSTACLE or succeed == COLLISION_WALL:
            reward -= 1500
            if(accel_count > Accel_Count_Th):
                reward -= 50
            accel_count = 0
        return float(reward)


def reward_initalize(init_distance_to_goal):
    global goal_dist_initial
    goal_dist_initial = init_distance_to_goal

function_name = "get_reward_" + REWARD_FUNCTION
reward_function_internal = globals()[function_name]
if reward_function_internal == None:
    quit(f"Error: reward function {function_name} does not exist")
