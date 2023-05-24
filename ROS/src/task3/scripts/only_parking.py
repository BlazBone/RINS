#!/usr/bin/python3

# DISABLE SHADOWS IN RINS_WORLD BEFORE STARTING THE PROGRAM - CLEARS THE IMAGE SIGNIFICANTLY

import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from tf.transformations import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, Quaternion, PoseWithCovarianceStamped, Twist
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Bool, String

# OUR IMPORTS
from tf2_geometry_msgs import PoseStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
from typing import Tuple


def get_angle(image):
    point = (image.shape[0], image.shape[1]//2)
    closes_point = find_closest_pixel(image)    
    # print(closes_point)
    return np.arctan2(closes_point[0]-point[0], closes_point[1]-point[1])

def find_closest_pixel(image):
    point = (image.shape[0], image.shape[1]//2)
    closes_point = None
    for i in range(0,image.shape[0]-50):
        for j in range(image.shape[1]):
            if image[i,j] == 0:
                if closes_point is None:
                    closes_point = (i,j)
                else:
                    if np.linalg.norm(np.array(point)-np.array(closes_point)) > np.linalg.norm(np.array(point)-np.array((i,j))):
                        closes_point = (i,j)
    return closes_point

def get_angle_furthers_point(image):

    point = (image.shape[0], image.shape[1]//2)
    furthers_point = None
    for i in range(0,image.shape[0]-50):
        for j in range(image.shape[1]):
            if image[i,j] == 0:
                furthers_point = (i,j)
                return np.arctan2(furthers_point[0]-point[0], furthers_point[1]-point[1])

    return None

def get_highest_black_pixel(image):
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            if image[i,j] == 0:
                return i, j
    return None

class Parking:
    def __init__(self):
        rospy.init_node('parking_node', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1
        


        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # OUR ATTRIBUTES

        self.simple_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        self.twist_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.arm_mover_pub = rospy.Publisher("/arm_command", String, queue_size=10)

        self.is_parked = False

        self.greeting_position_green_ring = None
        
    def get_binary_arm_image(self):
        try:
            # get extended camera image
            data = rospy.wait_for_message('/arm_camera/rgb/image_raw', Image)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # # cut 10 pixels from the top and bottom right and left
        gray = gray[10:gray.shape[0]-10, 10:gray.shape[1]-10]

        # copy and bin image
        img = gray.copy()
        img[img < 10] = 0
        img[img >= 10] = 255

        # if there is no black pixel, binarize again
        if not np.any(img == 0):
            img = gray.copy()
            img[img >= 200] = 0

        return img
    
    
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
    

    def publish_second_greeting_pose(self):
        green_pose = PoseStamped()
         
        green_pose.header.stamp = rospy.Time().now()
        green_pose.header.frame_id = "map"

        green_pose.pose.position.x = -0.6837977420946423
        green_pose.pose.position.y = 0.9226411301964432

        green_pose.pose.orientation.z = 0.6887448776174251
        green_pose.pose.orientation.w = 0.7250037886492443

        self.simple_goal_pub.publish(green_pose)
        print("PUBLISHED SECOND PRISON POSE")
        rospy.sleep(1)
        

    def fine_manouvering_rotation(self):
        img = self.get_binary_arm_image()
        
        if not np.any(img == 0):
            print("SECOND GREETING POSE")
            self.publish_second_greeting_pose()

        angle = -(get_angle(img) + np.pi/2)

        twist_msg = Twist()
        twist_msg.linear.x = 0.00
        twist_msg.angular.z = angle

        self.twist_pub.publish(twist_msg)
        print("ROTATED THE APROPRIATE AMOUNT")
        print("ANGLE IS ", angle)
        rospy.sleep(0.2)

        img = self.get_binary_arm_image() 

        while get_highest_black_pixel(img)[0] < 10:
            img = self.get_binary_arm_image()
            # angle = -(get_angle(img) + np.pi/2)
            #
            # twist_msg = Twist()
            # twist_msg.linear.x = 0.00
            # twist_msg.angular.z = angle
            #
            # self.twist_pub.publish(twist_msg)
            # rospy.sleep(0.2)
            # print("FIRST LOOP")
            # print(get_highest_black_pixel(img)[0])
            # cv2.imshow("arm_image", img)
            # cv2.waitKey(0)
            twist_msg = Twist()
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = 0.00
            self.twist_pub.publish(twist_msg)
            rospy.sleep(0.2)
        
        img = self.get_binary_arm_image()


        angle_furthest = -(get_angle_furthers_point(img) + np.pi/2)
        twist_msg = Twist()
        twist_msg.linear.x = 0.00
        twist_msg.angular.z = angle_furthest

        self.twist_pub.publish(twist_msg)
        print("ROTATED THE APROPRIATE AMOUNT")
        print("ANGLE IS ", angle_furthest)
        rospy.sleep(0.2)

        img = self.get_binary_arm_image() 
        highest_pixel = get_highest_black_pixel(img)[0]
        # print("HIGHEST PIXEL BEFORE 2ND LOOP")
        # print(highest_pixel)
        while get_highest_black_pixel(img)[0] < img.shape[0]-150:
            # print("SECOND LOOP")
            print(get_highest_black_pixel(img)[0])
            img = self.get_binary_arm_image()
            twist_msg = Twist()
            twist_msg.linear.x = 0.05
            twist_msg.angular.z = 0.00
            self.twist_pub.publish(twist_msg)
            rospy.sleep(0.2)
            angle_furthest = -(get_angle_furthers_point(img) + np.pi/2)
            twist_msg = Twist()
            twist_msg.linear.x = 0.00
            twist_msg.angular.z = angle_furthest

            self.twist_pub.publish(twist_msg)
            print("ROTATED THE APROPRIATE AMOUNT IN 2ND LOOP")
            print("ANGLE IS ", angle_furthest)
            rospy.sleep(0.2)


        self.is_parked = True

        print("PARKED IS TRUE")


def main():
    parking = Parking()
    rospy.sleep(1)
    parking_no_movement_pub = rospy.Publisher("/only_movement/parking_is_going_on", Bool, queue_size=10)
    rospy.sleep(0.5)
    rospy.loginfo("Starting the parking finder node")
    parking.greeting_position_green_ring = rospy.wait_for_message("/only_movement/parking_search", PoseStamped)    
    print("PARKING: GOT PRISON GREETING POSE", parking.greeting_position_green_ring)

    rospy.loginfo("STARTED PARKING")

    
    
    # PARKING NODE TAKES CONTROL OVER MOVEMENT
    parking_no_movement_msg = Bool()
    parking_no_movement_msg.data = True
    rospy.sleep(0.5)
    parking_no_movement_pub.publish(parking_no_movement_msg)

    parking_no_movement_msg = Bool()
    parking_no_movement_msg.data = True
    rospy.sleep(0.5)
    parking_no_movement_pub.publish(parking_no_movement_msg)

    rospy.sleep(1)
    
    # print(parking.greeting_position_green_ring)
    # parking.simple_goal_pub.publish(parking.greeting_position_green_ring)

    parking.publish_second_greeting_pose()
    print("PARKING: PUBLISHED 2nd PRISON GREETING POSE")
    print(parking.greeting_position_green_ring)

    while not parking.status_reached()[0]:
        pass

    parking.arm_mover_pub.publish("extend")

    rospy.sleep(2)
    print("STARTED FINE MANOUVERING")    
    parking.greeting_position_green_ring = None
    parking.fine_manouvering_rotation()
    while not rospy.is_shutdown() and not parking.is_parked:
        pass

    rospy.loginfo("Parking finder node finished.")

    parking.arm_mover_pub.publish("retract")

    # MOVEMENT NODE TAKES CONTROL OVER MOVEMENT
    parking_no_movement_msg = Bool()
    parking_no_movement_msg.data = False
    rospy.sleep(0.5)
    parking_no_movement_pub.publish(parking_no_movement_msg)



if __name__ == '__main__':
    main()
