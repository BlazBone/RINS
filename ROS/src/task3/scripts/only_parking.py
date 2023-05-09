#!/usr/bin/python3

# DISABLE SHADOWS IN RINS_WORLD BEFORE STARTING THE PROGRAM - CLEARS THE IMAGE SIGNIFICANTLY

from operator import contains
import sys
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
from std_msgs.msg import ColorRGBA, Bool
from matplotlib import pyplot as plt

# OUR IMPORTS
import os
import math
from exercise6_utils import read_path_log_orientation
from tf2_geometry_msgs import PoseStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
import shutil
from typing import List, Tuple
import time
from functools import reduce

PARKING_DIR = os.path.join(os.path.dirname(__file__), "../last_run_info/park_spaces_images/")

def get_angle(image):
    point = (image.shape[0], image.shape[1]//2)
    closes_point = find_closest_pixel(image)    
    print(closes_point)
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
        

        # Publisher for the visualization markers
        self.markers_pub = rospy.Publisher('markers3', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # OUR ATTRIBUTES

        self.simple_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.cancel_goal_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)

        self.twist_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)

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
    
    def get_pose(self,e,dist, time_stamp, marker_shape, marker_color, detected_object, detected_color):
        # parking_spaces are below rings - can share markers
        # if detected_object == "parking_space":
            # detected_object = "ring"
        # if detected_color.lower() not in ("red", "green", "blue", "black", "yellow", "white"):
            # skip 'unknown' color
            # return

        # Calculate the position of the detected ellipse
        k_f = 554 # kinect focal length in pixels

        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]

        angle_to_target = np.arctan2(elipse_x,k_f)
        
        # Get the angles in the base_link relative coordinate system
        x,y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        ### Define a stamped message for transformation - directly in "base_frame"
        #point_s = PointStamped()
        #point_s.point.x = x
        #point_s.point.y = y
        #point_s.point.z = 0.3
        #point_s.header.frame_id = "base_link"
        #point_s.header.stamp = rospy.Time(0)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        # point_s.header.stamp = rospy.Time(0) # tle damo current time have to fix
        point_s.header.stamp = time_stamp

        # Get the point in the "map" coordinate system
        point_world = self.tf_buf.transform(point_s, "map")

        # marker_coords = (point_world.point.x, point_world.point.y, point_world.point.z)
        # ALL_MARKER_COORDS[detected_object][detected_color].append(marker_coords)

        # print(f"added {detected_color.upper()} marker for {detected_object}!")
        # print(f"Current markers for color {detected_color} and object {detected_object} are: {len(ALL_MARKER_COORDS[detected_object][detected_color])}")

        # all_coordinates = np.array(ALL_MARKER_COORDS[detected_object][detected_color])
        # avg_x = np.mean(all_coordinates[:, 0])
        # avg_y = np.mean(all_coordinates[:, 1])
        # avg_z = np.mean(all_coordinates[:, 2])

        # Create a Pose object with the same position
        pose = Pose()
        pose.position.x = point_world.point.x
        pose.position.y = point_world.point.y
        pose.position.z = point_world.point.z

        # so we get no errors
        pose.orientation.z = 1
        pose.orientation.w = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        
        # PUBLISHING MARKER
        # Create a marker used for visualization
        self.marker_num += 1
        marker = Marker()
        marker.header.stamp = point_world.header.stamp
        marker.header.frame_id = point_world.header.frame_id
        marker.pose = pose
        # mybe we can place different markers for different objects (param in get pose or something)
        # so we can more 3easily destinguish them
        marker.type = marker_shape
        marker.action = Marker.ADD
        marker.frame_locked = False
        # i want to se markers all the time not only when we detect new ones
        marker.lifetime = rospy.Duration(1000) # this way marker stays up until deleted
        marker.id = self.marker_num
        marker.scale = Vector3(0.1, 0.1, 0.1)
                # mybe we can place different markers for different objects (param in get pose or something)
        # so we can more 3easily destinguish them
        # same with different colors
        marker.color = marker_color
        
        # BEST_MARKERS[detected_object][detected_color] = marker
        self.marker_array.markers.append(marker)
        # self.marker_array = get_marker_array_to_publish()
        #
        # markers_to_publish = get_marker_array_to_publish()
        self.markers_pub.publish(self.marker_array)
        # print(f"PUBLISHED MARKER ARRAY OF LEN {len(markers_to_publish.markers)}!")
    
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
    
    def in_circle(self):
        return False
    def publish_second_greeting_pose(self):
        # header: 
        #   seq: 202
        #   stamp: 
        #     secs: 1549
        #     nsecs: 627000000
        #   frame_id: "map"
        # pose: 
        #   pose: 
        #     position: 
        #       x: 2.091554422903801
        #       y: 0.3509621880975083
        #       z: 0.0
        #     orientation: 
        #       x: 0.0
        #       y: 0.0
        #       z: -0.9171601902651947
        #       w: 0.3985187390734743
        #   covariance: [0.0022698431204277725, -0.000405965448689094, 0.0, 0.0, 0.0, 0.0, -0.000405965448689094, 0.0026165707487205703, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.002780653885771563]
        green_pose = PoseStamped()
         
        green_pose.header.stamp = rospy.Time().now()
        green_pose.header.frame_id = "map"

        green_pose.pose.position.x = 2.091554422903801
        green_pose.pose.position.y = 0.3509621880975083

        green_pose.pose.orientation.z = -0.9171601902651947
        green_pose.pose.orientation.w = 0.3985187390734743

        self.simple_goal_pub.publish(green_pose)
        print("PUBLISHED SECOND GREEN POSE")
        rospy.sleep(5)
        

    def fine_manouvering_rotation(self):
        img = self.get_binary_arm_image()
        
        if not np.any(img == 0):
            print("SECOND GREETING POSE")
            self.publish_second_greeting_pose()

        angle = -(get_angle(img) + np.pi/2)

        # change to degrees
        twist_msg = Twist()
        twist_msg.linear.x = 0.00
        twist_msg.angular.z = angle

        self.twist_pub.publish(twist_msg)
        print("ROTATED THE APROPRIATE AMOUNT")
        print("ANGLE IS ", angle)
        rospy.sleep(0.2)

        img = self.get_binary_arm_image() 

        # cv2.imshow("arm_image", img)
        # cv2.waitKey(0)
        while get_highest_black_pixel(img)[0] < 10:
            print("FIRST LOOP")
            print(get_highest_black_pixel(img)[0])
            img = self.get_binary_arm_image()
            # cv2.imshow("arm_image", img)
            # cv2.waitKey(0)
            twist_msg = Twist()
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = 0.00
            self.twist_pub.publish(twist_msg)
            rospy.sleep(0.2)
        
        img = self.get_binary_arm_image()

        # cv2.imshow("arm_image", img)
        # cv2.waitKey(0)

        angle_furthest = -(get_angle_furthers_point(img) + np.pi/2)
        # change to degrees
        twist_msg = Twist()
        twist_msg.linear.x = 0.00
        twist_msg.angular.z = angle_furthest

        self.twist_pub.publish(twist_msg)
        print("ROTATED THE APROPRIATE AMOUNT")
        print("ANGLE IS ", angle_furthest)
        rospy.sleep(0.2)

        img = self.get_binary_arm_image() 
        highest_pixel = get_highest_black_pixel(img)[0]
        print("HIGHEST PIXEL BEFORE 2ND LOOP")
        print(highest_pixel)
        while get_highest_black_pixel(img)[0] < img.shape[0]-150:
            print("SECOND LOOP")
            print(get_highest_black_pixel(img)[0])
            img = self.get_binary_arm_image()
            twist_msg = Twist()
            twist_msg.linear.x = 0.05
            twist_msg.angular.z = 0.00
            self.twist_pub.publish(twist_msg)
            rospy.sleep(0.2)
            angle_furthest = -(get_angle_furthers_point(img) + np.pi/2)
            # change to degrees
            twist_msg = Twist()
            twist_msg.linear.x = 0.00
            twist_msg.angular.z = angle_furthest

            self.twist_pub.publish(twist_msg)
            print("ROTATED THE APROPRIATE AMOUNT IN 2ND LOOP")
            print("ANGLE IS ", angle_furthest)
            rospy.sleep(0.2)


        self.is_parked = True

        # i = 0
        # while i < 100:
        #     i+=1
        print("PARKED IS TRUE")



    def go_to_parking_space(self, e, dist, pose_of_detection):
        quaternion_array = self.get_parking_center_quaternion_array(e, dist, pose_of_detection)
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time().now()
        msg.pose.position.x = pose_of_detection.pose.pose.position.x
        msg.pose.position.y = pose_of_detection.pose.pose.position.y
        msg.pose.position.z = pose_of_detection.pose.pose.position.z
        msg.pose.orientation.x = quaternion_array[0]
        msg.pose.orientation.y = quaternion_array[1]
        msg.pose.orientation.z = quaternion_array[2]
        msg.pose.orientation.w = quaternion_array[3]
        
        self.simple_goal_pub.publish(msg)
        print("PUBLISHED ROTATION")
        print(msg)
        while not self.status_reached()[0]:
            pass
        print("CENTERED PARKING SPACE")
        
        rospy.sleep(2)

        twist_msg = Twist()
        # velocity
        twist_msg.linear.x = dist/2
        # angular velocity
        twist_msg.angular.z = 0.0

        self.twist_pub.publish(twist_msg)
        print("PUBLISHED TWIST")

        rospy.sleep(2)
        print("PARKED")
        
        if self.in_circle():
            self.is_parked = True
        else:
            self.parking_image_callback()

    def get_parking_center_quaternion_array(self, e, dist : float, pose_of_detection: PoseWithCovarianceStamped) -> Tuple[float,float,float,float]:
        # Calculate the position of the detected face
        k_f = 554 # kinect focal length in pixels

        elipse_x = self.dims[1] / 2 - e[0][0]
        angle_to_target = np.arctan2(elipse_x,k_f)
        
        # Get the angles in the base_link relative coordinate system

        # Define a quaternion for the rotation
        # Roatation is such that the image of the face is in the center of the robot view.
        q = Quaternion()
        q.x = 0
        q.y = 0
        q.z = math.sin(angle_to_target)
        q.w = math.cos(angle_to_target)

    
        q2 = pose_of_detection.pose.pose.orientation

        goal_quaternion_array = quaternion_multiply((q2.x, q2.y, q2.z, q2.w), (q.x, q.y, q.z, q.w))
        
        return goal_quaternion_array

    def parking_image_callback(self):
        try:
            data = rospy.wait_for_message('/camera/rgb/image_raw', Image)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            p = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        except CvBridgeError as e:
            print(e)

        # closer might meen more in sync
        try:
            depth_img = rospy.wait_for_message('/camera/depth/image_raw', Image)
        except Exception as e:
            print(e)

        # Set the dimensions of the image
        self.dims = cv_image.shape

        # Tranform image to gayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # take only the bottom half
        # gray = gray[self.dims[0]//2:, :]

        # Do histogram equlization
        img = cv2.equalizeHist(gray)

    
        thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 25)


        # Extract contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Example how to draw the contours, only for visualization purposes
        cv2.drawContours(img, contours, -1, (255, 0, 0), 3)

        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            #     print cnt
            #     print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)


        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            for m in range(n + 1, len(elps)):
                e1 = elps[n]
                e2 = elps[m]
                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                if dist < 5:
                    candidates.append((e1,e2))


        org = (20, 20)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.5
        cv2.putText(cv_image, f"image shape: {cv_image.shape}", org, font, fontScale, (0,0,0), 1)

        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)
        # cv2.imshow("Image window", img)
        # cv2.waitKey(0)
        # Extract the depth from the depth image
        for n,c in enumerate(candidates):
        # for c in candidates:
            # the centers of the ellipses
            # ellipses have the same centers
            e1 = c[0]
            e2 = c[1]
            
            # x: horizontal, y: vertical
            # a: horizontal axis, b: vertical axis
            # angle: rotation angle
            (x_e1, y_e1), (a_e1, b_e1), angle_e1 = e1
            (x_e2, y_e2), (a_e2, b_e2), angle_e2 = e2
            
            # due to poor design of cv2, dimensions out of scope aren't an error, but drawn by module
            x_e1 = x_e1 % cv_image.shape[1]
            y_e1 = y_e1 % cv_image.shape[0]
            x_e2 = x_e2 % cv_image.shape[1]
            y_e2 = y_e2 % cv_image.shape[0]
            # drawing the ellipses on the image
            cv2.ellipse(cv_image, e1, (0, 255, 0), 2)
            cv2.ellipse(cv_image, e2, (0, 255, 0), 2)

            
            # ROI - region of interest
            # small region around the centre to extract the color from
            left_x, right_x = int(x_e1-2), int(x_e1+2)
            left_y, right_y = int(y_e1-2), int(y_e1+2)
            roi = cv_image[left_y:right_y, left_x:right_x]
            # [:3] -> we do not need alpha channel
            average_color = tuple(int(channel) for channel in cv2.mean(roi))[:3]
            average_color_value = int(sum(average_color) / len(average_color))
            org = (330, 20 + (n+1)*20)
            cv2.putText(cv_image, f"average color: {average_color} [{average_color_value}]", org, font, fontScale, color=(0,0,0), thickness=1)
            
            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "16UC1")
            depth_time = depth_img.header.stamp

            y_threshold = int(cv_image.shape[0] / 2)

            if y_e1 > y_threshold and average_color_value > 240: # WORKS FOR 90% OF CASES
                # if we are looking for rings on the ground the way we see them from camera x-diff should be grater than y diff (elipse should be flat and long not tall and thin)
                image_name = f"{PARKING_DIR}{time.time()}.jpg"
                print("PARKING SPACE FOUND")
                # f string to only show 2 decimals of x_e1
                info_ellipse_string = f"a_e1: {a_e1:.2f} b_e1: {b_e1:.2f} a_e2: {a_e2:.2f} b_e2: {b_e2:.2f} ratio1: {a_e1 / b_e1:.2f} ratio2: {a_e2 / b_e2:.2f}"
                # org is bottom left of cv_image
                org = (20, cv_image.shape[0] - 20)
                (x_e1, y_e1), (a_e1, b_e1), angle_e1 = e1
                (x_e2, y_e2), (a_e2, b_e2), angle_e2 = e2
                # cv2.line(cv_image, (0, y_min), (cv_image.shape[1], y_min), (255,0,0), 1)
                # cv2.line(cv_image, (0, y_max), (cv_image.shape[1], y_max), (255,0,0), 1)
                # cv2.line(cv_image, (x_min, 0), (x_min, cv_image.shape[0]), (0,0,255), 1)
                # cv2.line(cv_image, (x_max, 0), (x_max, cv_image.shape[0]), (0,0,255), 1)
                x1max = int(x_e1 + b_e1/2)
                x1min = int(x_e1 - b_e1/2)
                y1max = int(y_e1 + a_e1/2)
                y1min = int(y_e1 - a_e1/2)
                # y1max = int(y_e1 + 30)
                # y1min = int(y_e1 - 30)
                cv2.line(cv_image, (x1max, 0), (x1max, cv_image.shape[0]), (0,0,255), 1)
                cv2.line(cv_image, (x1min, 0), (x1min, cv_image.shape[0]), (0,0,255), 1)

                cv2.line(cv_image, (0, y1max), (cv_image.shape[1], y1max), (255,0,0), 1)
                cv2.line(cv_image, (0, y1min), (cv_image.shape[1], y1min), (255,0,0), 1)

                cv2.putText(cv_image, info_ellipse_string, org, font, fontScale, (0,0,0), 1)
                
                print(f"distance is: {float(np.nanmean(depth_image[x1min:x1max,y1min:y1max]))}")

                self.get_pose(e1, float(np.nanmean(depth_image[x1min:x1max,y1min:y1max])), depth_time, Marker.SPHERE, ColorRGBA(1, 1, 1, 1), "parking_space", "white")
                # self.go_to_parking_space(e1, float(np.nanmean(depth_image[x1min:x1max,y1min:y1max])), p)
                
                cv2.imwrite(image_name, cv_image) 

                # self.greeting_position_green_ring = None
            if self.greeting_position_green_ring is not None:
                print("NOT PARKING SPACE")
                print(self.greeting_position_green_ring)
                self.simple_goal_pub.publish(self.greeting_position_green_ring)
                print("GOING TO GREEN RING, DELETING ITS POSITION")
                print("WAITING 5 SECONDS")
                rospy.sleep(5)
                self.greeting_position_green_ring = None

                while not self.status_reached()[0]:
                    rospy.sleep(.1)

                print("GREEN RING GREETING POSE REACHED")
                self.fine_manouvering_rotation()



def main():
    parking = Parking()
    rospy.sleep(1)
    parking_no_movement_pub = rospy.Publisher("/only_movement/parking_is_going_on", Bool, queue_size=10)
    rospy.sleep(0.5)
    rospy.loginfo("Starting the parking finder node")
    parking.greeting_position_green_ring = rospy.wait_for_message("/only_movement/parking_search", PoseStamped)    
    print("GOT GREEN RING GREETING POSE", parking.greeting_position_green_ring)

    rospy.loginfo("STARTED PARKING")

    
    
    # PARKING NODE TAKES CONTROL OVER MOVEMENT
    parking_no_movement_msg = Bool()
    parking_no_movement_msg.data = True
    parking_no_movement_pub.publish(parking_no_movement_msg)

    parking_no_movement_msg = Bool()
    parking_no_movement_msg.data = True
    parking_no_movement_pub.publish(parking_no_movement_msg)

    rospy.sleep(1)
    
    print(parking.greeting_position_green_ring)
    parking.simple_goal_pub.publish(parking.greeting_position_green_ring)
    print("(IN MAIN) GOING TO GREEN RING, DELETING ITS POSITION")
    print("WAITING 5 SECONDS")
    rospy.sleep(5)
    parking.greeting_position_green_ring = None
    parking.fine_manouvering_rotation()
    while not rospy.is_shutdown() and not parking.is_parked:
        pass

    rospy.loginfo("Parking finder node finished.")

    # MOVEMENT NODE TAKES CONTROL OVER MOVEMENT
    parking_no_movement_msg = Bool()
    parking_no_movement_msg.data = False
    parking_no_movement_pub.publish(parking_no_movement_msg)



if __name__ == '__main__':
    main()

# header: 
#   seq: 202
#   stamp: 
#     secs: 1549
#     nsecs: 627000000
#   frame_id: "map"
# pose: 
#   pose: 
#     position: 
#       x: 2.091554422903801
#       y: 0.3509621880975083
#       z: 0.0
#     orientation: 
#       x: 0.0
#       y: 0.0
#       z: -0.9171601902651947
#       w: 0.3985187390734743
#   covariance: [0.0022698431204277725, -0.000405965448689094, 0.0, 0.0, 0.0, 0.0, -0.000405965448689094, 0.0026165707487205703, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.002780653885771563]
# ---
#
