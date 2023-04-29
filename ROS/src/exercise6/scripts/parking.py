#!/usr/bin/python3

# DISABLE SHADOWS IN RINS_WORLD BEFORE STARTING THE PROGRAM - CLEARS THE IMAGE SIGNIFICANTLY

import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# OUR IMPORTS
import os
from exercise6_utils import read_path_log_orientation
from tf2_geometry_msgs import PoseStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
import shutil
from typing import List, Tuple
import time

relative_path_to_additional_info = os.path.join(os.path.dirname(__file__), f"../last_run_info/")
dir_irregular = os.path.join(os.path.dirname(__file__), "../last_run_info/irregular")
dir_rings = os.path.join(os.path.dirname(__file__), "../last_run_info/rings_images/")
dir_cylinders = os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders_images/")
dir_park_spaces = os.path.join(os.path.dirname(__file__), "../last_run_info/park_spaces_images/")

class The_Ring:
    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image and/or depth topic
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback)

        # Publisher for the visualization markers
        self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # OUR ATTRIBUTES
        points_path = os.path.join(os.path.dirname(__file__), "newpoints.txt")
        self.path = read_path_log_orientation(points_path)
        self.path_idx = 0

        self.simple_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.cancel_goal_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)

    
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
        if log:
            self.simple_goal_pub.publish(msg)
            rospy.loginfo(f"Visiting POINT @ index {self.path_idx} in path.")
            rospy.loginfo(f"\n{msg.pose}")

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


    def get_pose(self,e,dist, marker_shape, marker_color):
        # Calculate the position of the detected ellipse

        k_f = 525 # kinect focal length in pixels

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
        point_s.header.stamp = rospy.Time(0)

        # Get the point in the "map" coordinate system
        point_world = self.tf_buf.transform(point_s, "map")

        # Create a Pose object with the same position
        pose = Pose()
        pose.position.x = point_world.point.x
        pose.position.y = point_world.point.y
        pose.position.z = point_world.point.z
        
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
        marker.lifetime = rospy.Duration.from_sec(1000)
        marker.id = self.marker_num
        marker.scale = Vector3(0.1, 0.1, 0.1)
                # mybe we can place different markers for different objects (param in get pose or something)
        # so we can more 3easily destinguish them
        # same with different colors
        # marker.color = ColorRGBA(0, 1, 0, 1)
        marker.color = marker_color
        self.marker_array.markers.append(marker)

        self.markers_pub.publish(self.marker_array)


    def image_callback_2(self):
        # print('I got a new image!')
        data = rospy.wait_for_message('/camera/rgb/image_raw', Image)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
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

        # Do histogram equlization
        img = cv2.equalizeHist(gray)

        # Binarize the image, there are different ways to do it
        #ret, thresh = cv2.threshold(img, 50, 255, 0)
        # ret, thresh = cv2.threshold(img, 70, 255, cv2.THRESH_BINARY)
        # this one seeems to work best
        thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 25)

        # Extract contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Example how to draw the contours, only for visualization purposes
        cv2.drawContours(img, contours, -1, (255, 0, 0), 3)
        cv2.imshow("Contour window",img)
        cv2.waitKey(1)

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

    # watafak ZAKAJ IMAMO PLUS 20?
            ellipse_center = (int(x_e1) + 20, int(y_e1))
            
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
            
            # FOR DEBUGGING PURPOSES ONLY - BLUE CIRCLE IS IN THE WAY OF COLOR RECOGNITION
            # cv2.circle(cv_image, (int(x_e1), int(y_e1)), 5, (255, 0, 0), -1)
            # font = cv2.FONT_HERSHEY_SIMPLEX
            # fontScale = 0.5
            # cv2.putText(cv_image, f"({int(x_e1)}, {int(x_e2)})", ellipse_center, font, fontScale, color=(255,0,0), thickness=1)

            # two extreme points of the ellipse region, that allow the drawing of the ellipse
            inside_rectangle = ((int(x_e1 - a_e1 / 2), int(y_e1 - b_e1 / 2)), (int(x_e1 + a_e1 / 2), int(y_e1 + b_e1 / 2))) 
            outside_rectangle = ((int(x_e2 - a_e2 / 2), int(y_e2 - b_e2 / 2)), (int(x_e2 + a_e2 / 2), int(y_e2 + b_e2 / 2)))
            
            # DEPRACTED, NOT REALLY USABLE
            # inside_to_outside_ratio_x = (outside_rectangle[0][0] - inside_rectangle[0][0]) / (outside_rectangle[1][0] - inside_rectangle[1][0]) if outside_rectangle[1][0] != inside_rectangle[1][0] else "OUTSIDE AND INSIDE RECTANGLES ARE THE SAME"
            # inside_to_outside_ratio_y = (outside_rectangle[0][1] - inside_rectangle[0][1]) / (outside_rectangle[1][1] - inside_rectangle[1][1]) if outside_rectangle[1][1] != inside_rectangle[1][1] else "OUTSIDE AND INSIDE RECTANGLES ARE THE SAME"
            # print(f"inside_to_outside_ratio_x: {inside_to_outside_ratio_x}")
            # print(f"inside_to_outside_ratio_y: {inside_to_outside_ratio_y}")

            inside_x_to_y = round((inside_rectangle[1][0] - inside_rectangle[0][0]) / (inside_rectangle[1][1] - inside_rectangle[0][1]), 2) if inside_rectangle[1][1] != inside_rectangle[0][1] else "INSIDE RECTANGLE IS A SQUARE"
            outside_x_to_y = round((outside_rectangle[1][0] - outside_rectangle[0][0]) / (outside_rectangle[1][1] - outside_rectangle[0][1]), 2) if outside_rectangle[1][1] != outside_rectangle[0][1] else "OUTSIDE RECTANGLE IS A SQUARE"
            # org1 = (20, 470 - (n+1)*40)
            # org2 = (20, 450 - (n+1)*40)
            # cv2.putText(cv_image, f"inside x to y: {inside_x_to_y}", org1, font, fontScale, color=(0,0,0), thickness=1)
            # cv2.putText(cv_image, f"outside x to y: {outside_x_to_y}", org2, font, fontScale, color=(0,0,0), thickness=1)

            # draw the elipse region rectangles (white)
            # cv2.rectangle(cv_image, inside_rectangle[0], inside_rectangle[1], (255, 255, 255), 2)
            # cv2.rectangle(cv_image, outside_rectangle[0], outside_rectangle[1], (255, 255, 255), 2)

            # size: average of the two axes
            # center: center of the ellipse (y,x)
            size_e1 = (e1[1][0]+e1[1][1])/2
            size_e2 = (e2[1][0]+e2[1][1])/2

            # takes bigger of the 2 concentric elipses
            # size = max(size_e1, size_e2)
            size = size_e1
            # these two should be fairly close
            center_e1 = (e1[0][1], e1[0][0])
            center_e2 = (e2[0][1], e2[0][0])

            # not really sure if this part matters, but just to be sure
            center = center_e1 if size == size_e1 else center_e2

            # TODO CHECK IF cv_image.shape[0] is x and cv_image.shape[1] is y
            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            # x_min = x1 if x1>0 else 0
            # x_max = x2 if x2 < cv_image.shape[1] else cv_image.shape[1]
            x_min = max(0, x1 % cv_image.shape[1])
            x_max = min(x2 % cv_image.shape[1], cv_image.shape[1]-1) # -1 so it doesn't raise errors

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            # y_min = y1 if y1 > 0 else 0
            # y_max = y2 if y2 < cv_image.shape[0] else cv_image.shape[0]
            y_min = max(0, y1 % cv_image.shape[0])
            y_max = min(y2 % cv_image.shape[0], cv_image.shape[0]-1) # -1 so it doesn't raise errors
 
            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "16UC1")
            try:
                # dist_l = np.mean(depth_image[x_min,y_min:y_max])
                # dist_r = np.mean(depth_image[x_max,y_min:y_max])

                dist_l = np.mean(depth_image[y_min:y_max,x_min])
                dist_r = np.mean(depth_image[y_min:y_max,x_max])
            except IndexError:
                print(f"IndexError: x_min: {x_min}, x_max: {x_max}, y_min: {y_min}, y_max: {y_max}, shape: {depth_image.shape}")
                return

            
            x1_center = int(center[0] - size / 4)
            x2_center = int(center[0] + size / 4)
            # x_min_center = x1_center if x1_center>0 else 0
            # x_max_center = x2_center if x2_center<cv_image.shape[1] else cv_image.shape[1]
            x_min_center = max(0, x1_center % cv_image.shape[1])
            x_max_center = min(x2_center % cv_image.shape[1], cv_image.shape[1]-1) # -1 so it doesn't raise errors

            y1_center = int(center[1] - size / 4)
            y2_center = int(center[1] + size / 4)
            # y_min_center = y1_center if y1_center > 0 else 0
            # y_max_center = y2_center if y2_center < cv_image.shape[0] else cv_image.shape[0]
            y_min_center = max(0, y1_center % cv_image.shape[0])
            y_max_center = min(y2_center % cv_image.shape[0], cv_image.shape[0]-1) # -1 so it doesn't raise errors
            
            # average pixel distance in the center of the ring
            dist_center = np.mean(depth_image[x_min_center:x_max_center,y_min_center:y_max_center])
            float(np.mean(depth_image[x_min:x_max,y_min:y_max]))
            org = (20, 40)
            cv2.putText(cv_image, f"dist_center: {round(dist_center,2)}", org, font, fontScale, color=(0,0,0), thickness=1)

            distance_difference = round(abs(dist_center - max(dist_l, dist_r)), 2)

            # org = (20, 40)
            org = (20, 40 + (n+1)*20)
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 0.5
            cv2.putText(cv_image, f"distance_difference: {distance_difference}", org, font, fontScale, (0,0,0), 1)

            # rings should be in the upper third
            # should be int - represents pixels
            y_threshold = int(cv_image.shape[0] / 2)
            # depracted - calculating by center now
            # highest_vertical = min(outside_rectangle[0][1], outside_rectangle[1][1])
            # lowest_vertical = max(outside_rectangle[0][1], outside_rectangle[1][1])
            
            # no need to log this, but if we need it again, here's the snippet
            # start_point = (0, lowest_vertical)
            # end_point =  (cv_image.shape[1], lowest_vertical)
            # color = (0,0,0)
            # thickness = 1
            # cv2.line(cv_image, start_point, end_point, color, thickness)
            # org = (start_point[0], start_point[1] + 20)
            # font = cv2.FONT_HERSHEY_SIMPLEX
            # fontScale = 0.5
            # cv2.putText(cv_image, f"Lowest vertical: {lowest_vertical}", org, font, fontScale, color, thickness)

            # start_point = (0, highest_vertical)
            # end_point =  (cv_image.shape[1], highest_vertical)
            # color = (255,255,255)
            # thickness = 1
            # cv2.line(cv_image, start_point, end_point, color, thickness)
            # org = (start_point[0], start_point[1] + 20)
            # font = cv2.FONT_HERSHEY_SIMPLEX
            # fontScale = 0.5
            # cv2.putText(cv_image, f"Highest vertical: {highest_vertical}", org, font, fontScale, color, thickness)



            # logging of values moved to the image as it's more clear
            # --------------------------------------------------------
            # print(f"OUTSIDE RECTANGLE: {outside_rectangle}")
            # print(f"LOWEST VERTICAL: {lowest_vertical}")
            # print(f"HIGHEST VERTICAL: {highest_vertical}")
            # print(f"cv_image.shape: {cv_image.shape}")
            # print(f"y_threshold: {y_threshold}")

            # if dist_center > max(dist_l, dist_r):
            # if lowest_vertical > y_threshold:

            # THIS SHOULD BE MOVED TO A BOOLEAN FUNCTION FOR CLARITY
            distance_to_background = abs(average_color_value - 118)
            distance_to_no_background = abs(average_color_value - 178)
            # MYBE COMBINE THIS two distance_to_background -  distance_to_no_background > treshold? 

            ## distance difference mybe should be grater that 1 or 0.5 or something... 0 seems to be wrong since the ring can be tilted or the depth image is not rly accurate not sure 
            if ellipse_center[1] < y_threshold and distance_difference > 0 and min(inside_x_to_y, outside_x_to_y) > 0.75 and max(average_color) < 180 and min(average_color) > 100 and distance_to_background > distance_to_no_background:
                image_name = f"{dir_rings}{time.time()}.jpg"
                print("Found a ring!")
                # print(f"DEPTH AT CENTER: {dist_center}")
                # print(f"Image shape: {cv_image.shape}")
                # print(f"y_threshold: {y_threshold}")
                # print(f"min(y_min, y_max): {min(y_min, y_max)}")
                
                # center line - threshold of what should be recognized and what shouldn't
                start_point = (0, y_threshold)
                end_point =  (cv_image.shape[1], y_threshold)
                color = (0,255,0)
                thickness = 1
                cv2.line(cv_image, start_point, end_point, color, thickness)

                org = (start_point[0], start_point[1] + 20)
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 0.5
                cv2.putText(cv_image, f"threshold: {y_threshold}", org, font, fontScale, color, thickness)
                

                cv2.imshow("Image window", cv_image)
                cv2.imwrite(image_name, cv_image) # Save the face image

                x1max = int(x_e1 + b_e1/2)
                x1min = int(x_e1 - b_e1/2)
                y1max = int(y_e1 + a_e1/2)
                y1min = int(y_e1 - a_e1/2)
                self.get_pose(e1, float(np.nanmean(depth_image[x1min:x1max,y1min:y1max])), Marker.CUBE, ColorRGBA(0, 1, 0, 1))
            elif ellipse_center[1] >= y_threshold and min(inside_x_to_y, outside_x_to_y) > 0.75:
                image_name = f"{dir_cylinders}{time.time()}.jpg"
                print("WROTE CYLINDER")
                cv2.imwrite(image_name, cv_image) # Save the face image
                x1max = int(x_e1 + b_e1/2)
                x1min = int(x_e1 - b_e1/2)
                y1max = int(y_e1 + a_e1/2)
                y1min = int(y_e1 - a_e1/2)
                self.get_pose(e1, float(np.nanmean(depth_image[x1min:x1max,y1min:y1max])), Marker.CYLINDER, ColorRGBA(1, 0, 0, 1))
                # ration between two elipses must be same 
            elif ellipse_center[1] > y_threshold and abs((a_e1 / b_e1) - (a_e2 / b_e2)) < 0.07 and a_e1 / b_e1 < 0.4 and average_color_value > 240:
                # abs((a_e1 / b_e1) - (a_e2 / b_e2)) < 0.07 two elipses should be simmilar shape
                # a_e1 / b_e1 < 0.4 a (vertical should be a lot smaller)
            # elif ellipse_center[1] > y_threshold and average_color_value > 240: # WORKS FOR 90% OF CASES
                # if we are looking for rings on the ground the way we see them from camera x-diff should be grater than y diff (elipse should be flat and long not tall and thin)
                image_name = f"{dir_park_spaces}{time.time()}.jpg"
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
                self.get_pose(e1, float(np.nanmean(depth_image[x1min:x1max,y1min:y1max])), Marker.SPHERE, ColorRGBA(0, 0, 1, 1))
                # y1max = int(y_e1 + 30)
                # y1min = int(y_e1 - 30)
                cv2.line(cv_image, (x1max, 0), (x1max, cv_image.shape[0]), (0,0,255), 1)
                cv2.line(cv_image, (x1min, 0), (x1min, cv_image.shape[0]), (0,0,255), 1)

                cv2.line(cv_image, (0, y1max), (cv_image.shape[1], y1max), (255,0,0), 1)
                cv2.line(cv_image, (0, y1min), (cv_image.shape[1], y1min), (255,0,0), 1)

                cv2.putText(cv_image, info_ellipse_string, org, font, fontScale, (0,0,0), 1)
                self.get_pose(e1, float(np.nanmean(depth_image[x1min:x1max,y1min:y1max])), Marker.SPHERE, ColorRGBA(0, 0, 1, 1))
                
                cv2.imwrite(image_name, cv_image) 
                # print(f"Skipping image, dist_center: {dist_center}, y_threshold: {y_threshold}, center[1]: {center[1]}")
                # print(f"Skipping image, distance: {dist}, type: {type(dist)}")
            
            # calculates position of detected object and published position to marker? (the position howvevr in my eexperience is not accurate)

        # if len(candidates)>0:
        #         cv2.imshow("Image window",cv_image)
        #         cv2.waitKey(1)

    def depth_callback(self,data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

        # Do the necessairy conversion so we can visuzalize it in OpenCV
        image_1 = depth_image / 65536.0 * 255
        image_1 =image_1/np.max(image_1)*255

        image_viz = np.array(image_1, dtype= np.uint8)

        cv2.imshow("Depth window", image_viz)
        cv2.waitKey(1)


def main():
    for path in (relative_path_to_additional_info, dir_irregular, dir_rings, dir_cylinders, dir_park_spaces):
        if os.path.exists(path):
            shutil.rmtree(path)
        os.mkdir(path)

    ring_finder = The_Ring()
    
    rate = rospy.Rate(10)
        
    rospy.sleep(1)
    ring_finder.publish_new_position()                

    while not rospy.is_shutdown():
        reached, status = ring_finder.status_reached()
        ring_finder.image_callback_2()
        if reached:
            # no need to implement anything more sophisticated for exercise 6
            message = "REACHED GOAL" if status == 3 else "CANCEL STATUS"
            rospy.loginfo(message)
            ring_finder.publish_new_position()

        elif ring_finder.path_idx == len(ring_finder.path):
            rospy.loginfo("FINISHED PATH")
            break

        rate.sleep()
    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
