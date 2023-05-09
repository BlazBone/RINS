#!/usr/bin/python3

# DISABLE SHADOWS IN RINS_WORLD BEFORE STARTING THE PROGRAM - CLEARS THE IMAGE SIGNIFICANTLY

# from operator import contains
import rospy
import tf2_geometry_msgs
import tf2_ros
from std_msgs.msg import String, Bool

# OUR IMPORTS
import os
from exercise6_utils import read_path_log_orientation
from tf2_geometry_msgs import PoseStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
from typing import Tuple
import time

class Movement:
    def __init__(self):
        rospy.init_node('movement_node', anonymous=True)

        # OUR ATTRIBUTES

        points_path = os.path.join(os.path.dirname(__file__), "./paths/task2_points.txt")
        self.path = read_path_log_orientation(points_path)
        self.path_idx = 0

        self.simple_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.cancel_goal_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)

        self.arm_mover_pub = rospy.Publisher("/arm_command", String, queue_size=10)
        
        self.park_sub = rospy.Subscriber("/only_movement/parking_is_going_on", Bool, self.park_callback)
        self.currently_parking = False

        self.greeting_sub = rospy.Subscriber("/only_movement/greeting_is_going_on", Bool, self.greet_callback)
        self.currently_greeting = False

        

    def cancel_goal(self):
        cancel_msg = GoalID()
        self.cancel_goal_pub.publish(cancel_msg)

    def greet_callback(self, data):
        print("GREETING CALLBACK")
        print(data)
        
        self.currently_greeting = data.data
        
        print(self.currently_greeting)

        self.cancel_goal()
        
        self.path_idx = max(0, self.path_idx-1)

    
    def park_callback(self, data):
        print("PARKING CALLBACK")
        print(data)
        self.currently_parking = data.data
        print(self.currently_parking)
        
        self.cancel_goal()
        self.path_idx = max(0, self.path_idx-1)
        print("CANCELED GOAL, PARKING IS TAKING OVER")
        if data.data:
            self.arm_mover_pub.publish("extend")
        else:
            self.arm_mover_pub.publish("retract")


    def publish_new_position(self, log:bool=True) -> None:
        """
        Publishes a new position to the robot.

        Go to path_idx.
        Function updates self.path_idx.
        """
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time().now()
        if self.path_idx >= len(self.path):
            return
        msg.pose.orientation.w = 1
        msg.pose.position.x = self.path[self.path_idx][0]
        msg.pose.position.y = self.path[self.path_idx][1]
        msg.pose.orientation.z = self.path[self.path_idx][3]
        msg.pose.orientation.w = self.path[self.path_idx][4]
        self.path_idx += 1
        self.simple_goal_pub.publish(msg)
        if log:
            rospy.loginfo(f"Visiting POINT @ index {self.path_idx} in path.")

    def status_reached(self) -> Tuple[bool, int]:
        """
        Function listenes for status updates on topic /move_base/status.

        If status is 3 (goal reached) or 4 (goal terminated), the goal is 'reached'.
        If status is 0 or 1 the goal is 'being processed', else there is an error.
        """
        status = rospy.wait_for_message("/move_base/status", GoalStatusArray)
        if status.status_list[-1].status in (0, 1):
            # goal is being processed
            return False,status.status_list[-1].status
        else:
            # goal is done (successfully or not)
            return True,status.status_list[-1].status



def main():
    movement = Movement()
    rospy.loginfo("Starting the movement node")
    rospy.sleep(4)
    
            
    movement.publish_new_position()                
    loop_time = 0
    loop_count = 0


    while not rospy.is_shutdown():
        start_time = time.time()
        loop_count += 1
        reached, status = movement.status_reached()
        
        if movement.currently_greeting or movement.currently_parking:
            continue
        
        if reached:
            # no need to implement anything more sophisticated for exercise 6
            message = "REACHED GOAL" if status == 3 else "CANCEL STATUS"
            rospy.loginfo(message)
            movement.publish_new_position()

        elif movement.path_idx == len(movement.path):
            rospy.loginfo("FINISHED PATH")
            # send signal to all nodes
            end_publisher = rospy.Publisher("/end", Bool, queue_size=1)
            msg_something = Bool()
            msg_something.data = True
            end_publisher.publish(msg_something)
            time.sleep(5)
            # sleep mybe to finish processing images.
            break
        loop_time += time.time() - start_time
    
    print(f"Average loop time: {loop_time / loop_count}")


if __name__ == '__main__':
    main()
