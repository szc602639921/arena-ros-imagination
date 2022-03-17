#!/usr/bin/env python

from typing import Counter
import rospy
import json
from nav_msgs.msg import Odometry, OccupancyGrid
from rospy.topics import Publisher
from tf.transformations import euler_from_quaternion # to make it work with python3 do: cd $HOME/geometry2_ws $ source devel/setup.bash $ cd $HOME/catkin_ws_ma $ source devel/setup.bash
from geometry_msgs.msg import Point, Twist, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
  
import cv2 # does not work in (rosnav)
import rospkg
import os
import numpy as np
from numpy import asarray
from std_msgs.msg import String
import time
import actionlib
# please change the root path
root_path = "/home/henry/arena_ws/src/"

x_global = 0.0
y_global = 0.0 
theta_global = 0.0

time_start = 0.0
temp_time = 0.0
img_sec = 1 # default

start_path = 0
final_path = 0
paths_between_start_and_end = 1 # easy change 0 vs. 1

move_base_goal = MoveBaseGoal()
goal_global = PoseStamped()
position_global = Point()
position_global.x = 0.0
position_global.y = 0.0
position_global.z = 0.0
orientation_global = Quaternion()
sub_goal = []
global_speed = Twist()

def movebase_client(x, y, goal_pub):
    goal = MoveBaseGoal()
    goal.target_pose.header.seq = 0
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    # TODO: why both publish() and actionlib? (visualize the arrow vs. moves the robot?)
    pose_stamped = PoseStamped()
    pose_stamped = goal.target_pose
    goal_pub.publish(pose_stamped)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    client.send_goal(goal)

def callback_odom(data):
    global position_global, orientation_global
    position_global = data.pose.pose.position
    orientation_global = data.pose.pose.orientation

# read scenario config file
def main():
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    senairo_path = "arena-rosnav/simulator_setup/training/scenario1.json"
    stream = open(root_path + senairo_path, 'r')
    config = json.load(stream)
    print('json file content:\n' + str(config) + '\n')
    num_images = config['num_images']
    img_sec = int(1.0/(num_images/60.0))
    # sub_pair_map_60x60 = rospy.Subscriber("pair_temp_60x60", ListOccupancyGrid, callback_pair_temp_60x60)
    for path in config['robot_paths']:
        pub_counter = rospy.Publisher("/imagination_counter", String, queue_size=10)
        pub_counter.publish(str(0))
        # movebase_client(initial_pos[0], initial_pos[1], goal_pub)
        for subgoal in path['subgoals']:
            rospy.Subscriber("/odom", Odometry, callback_odom)
            pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
            robot_pos = np.array([position_global.x, position_global.y])
            goal_distance = np.linalg.norm(robot_pos-subgoal)
            radius = 0.15
            movebase_client(subgoal[0], subgoal[1], goal_pub)
            Counter = 0
            while goal_distance > radius*(1 + Counter/100):
                Counter +=1
                robot_pos = np.array([position_global.x, position_global.y])
                goal_distance = np.linalg.norm(robot_pos-subgoal)
                rospy.Subscriber("/odom", Odometry, callback_odom)
                if Counter%15 == 0:
                    print('set the goal again')
                    movebase_client(subgoal[0], subgoal[1], goal_pub)
                
                time.sleep(0.1)
                
                
                
def callback_timer(data):
    global timer_done
    timer_done = data.data

def callback_map(map_data):
    map_data_array = asarray([map_data.data])

    free_amount = np.sum(map_data_array[0] == 0)
    unknown_amount = np.sum(map_data_array[0] == -1)
    ocupied_amount = np.sum(map_data_array[0] == 100)

    rospack = rospkg.RosPack()
    map_data_array2 = np.array(map_data.data) # array of size 346986
    relative_img_path = os.path.join(rospack.get_path("simulator_setup"), "maps", "map_empty", "map_small.png")
    used_map_image = cv2.imread(relative_img_path) # get the size of the used map image: width x height 666 x 521
    map_shape = (used_map_image.shape[0],used_map_image.shape[1])
    map_reshaped = map_data_array2.reshape(map_shape)
    temp = np.ones(map_reshaped.shape)*255
    temp[map_reshaped != -1] = map_reshaped[map_reshaped != -1]

    cv2.imwrite("map_topic.png", temp) # will be saved 

if __name__ == '__main__':
    rospy.init_node('reach_goal', anonymous=True)
    rospy.Subscriber("/costmap_timer_done", String, callback_timer)
    rospy.Subscriber('/map', OccupancyGrid, callback_map)
    main()


