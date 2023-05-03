#!/usr/bin/python3

# DISABLE SHADOWS IN RINS_WORLD BEFORE STARTING THE PROGRAM - CLEARS THE IMAGE SIGNIFICANTLY

from operator import contains
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
from std_msgs.msg import ColorRGBA, Bool
from matplotlib import pyplot as plt

# OUR IMPORTS
import os
from exercise6_utils import read_path_log_orientation
from tf2_geometry_msgs import PoseStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
import shutil
from typing import List, Tuple
import time
from functools import reduce

dirs = {
        "dir_park_spaces" : os.path.join(os.path.dirname(__file__), "../last_run_info/park_spaces_images/"),
        }

# calculated markers for all possible objects
ALL_MARKER_COORDS= {
    "cylinder": {
        "red": list(),
        "blue": list(),
        "green": list(),
        "yellow": list(),
        },
    # if rings are above parking spaces, there's no need to include them in this dict
    "ring": {
        "red": list(),
        "blue": list(),
        "green": list(),
        "black": list(),
        "white": list(), # currently only used for parking spaces
        },
    }

# best calculated markers for all possible objects
BEST_MARKERS= {
    "cylinder": {
        "red": None,
        "blue": None,
        "green": None,
        "yellow": None,
        },
    # if rings are above parking spaces, there's no need to include them in this dict
    "ring": {
        "red": None,
        "blue": None,
        "green": None,
        "black": None,
        "white": None, # currently only used for parking spaces
        },
    }

def get_marker_array_to_publish():
    marker_array = MarkerArray()
    for shape in BEST_MARKERS:
        for color in BEST_MARKERS[shape]:
            if BEST_MARKERS[shape][color]:
                marker_array.markers.append(BEST_MARKERS[shape][color])
    return marker_array

COLOR_DICT = {
        # COLORNAME: (lower_value, upper_value)
        "blue": (
               np.array([100, 50, 50]),
               np.array([130, 255, 255])
            ),
        "red": (
               np.array([0, 50, 50]),
               np.array([10, 255, 255])
            ),
        "green": (
               np.array([50, 50, 50]),
               np.array([70, 255, 255])
            ),
        "yellow": (
               np.array([20, 100, 100]),
               np.array([40, 255, 255])
            )
        }


class The_Ring:
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
        points_path = os.path.join(os.path.dirname(__file__), "newpoints.txt")
        self.path = read_path_log_orientation(points_path)
        self.path_idx = 0

        self.simple_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.cancel_goal_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)

    
    
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



    def image_callback(self):
        try:
            data = rospy.wait_for_message('/camera/rgb/image_raw', Image)
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
                image_name = f"{dirs['dir_park_spaces']}{time.time()}.jpg"
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
                
                cv2.imwrite(image_name, cv_image) 
                # image has no ring we skip it



def main():
    rospy.loginfo("Starting the parking finder node")
    # for dirname, path in dirs.items():
    #     if "dir" in dirname:
    #         if os.path.exists(path):
    #             shutil.rmtree(path)
    #         os.mkdir(path)
    #     else:
    #         for color in dirs[dirname]:
    #             if os.path.exists(dirs[dirname][color]):
    #                 shutil.rmtree(dirs[dirname][color])
    #             os.mkdir(dirs[dirname][color])

    ring_finder = The_Ring()

    rate = rospy.Rate(10)
        
    rospy.sleep(2)

    loop_time = 0
    loop_count = 0
    while not rospy.is_shutdown():
        start_time = time.time()
        ring_finder.image_callback()
        loop_count += 1
        loop_time += time.time() - start_time
        rate.sleep()

    print(f"Average loop time: {loop_time / loop_count}")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
