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
from std_msgs.msg import ColorRGBA
from matplotlib import pyplot as plt

# OUR IMPORTS

import os
from exercise6_utils import read_path_log_orientation
from tf2_geometry_msgs import PoseStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
import shutil
from typing import List, Tuple
import time
from std_msgs.msg import Bool
from functools import reduce

dirs = {
        "dir_cylinders" : os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/"),
        "cylinders": {
            "red": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/red/"),
            "blue": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/blue/"),
            "green": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/green/"),
            "yellow": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/yellow/"),
            },
        
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

# arbitrarily set, seems to be a good filter
MINIMAL_ACCEPTED_COLOR_PERCENTAGE = 3
MINIMAL_ACCEPTED_AREA = 5000

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

def apply_colour_mask(cv_image, hsv_image, show=False):
        
        IMAGE_EXTRACTED_DATA = {}
        
        all_pixels = cv_image.shape[0] * cv_image.shape[1]
        
        color_mask = None
        highest_percentage = 0
        for color in COLOR_DICT:
            # create a mask for each color separately
            mask = cv2.inRange(hsv_image, COLOR_DICT[color][0], COLOR_DICT[color][1])
            contains_colour = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            if contains_colour.any():
                mask_pixels = cv2.countNonZero(mask)
                percentage = round(mask_pixels / all_pixels * 100, 2)
                if percentage >= highest_percentage:
                    color_mask = mask
                    highest_percentage = percentage
                MASK_INFO = {color: (mask, percentage)}
                IMAGE_EXTRACTED_DATA.update(MASK_INFO)
        
        if IMAGE_EXTRACTED_DATA: 
            contains_color = True
            # color_mask = reduce(cv2.bitwise_or, [IMAGE_EXTRACTED_DATA[color][0] for color in IMAGE_EXTRACTED_DATA])
            # sort IMAGE_EXTRACTED_DATA decrasingly by percentages of colours in the image
            IMAGE_EXTRACTED_DATA = dict(sorted(IMAGE_EXTRACTED_DATA.items(), key=lambda x: x[1][1], reverse=True))
            # show the final color_mask
            if show:
                cv2.imshow('color_mask', color_mask)
                cv2.waitKey(1)
            return (contains_color, color_mask, IMAGE_EXTRACTED_DATA)
        else:
            contains_color = False
            return (contains_color, None, None)

def is_mask_cylinder(cv_image, mask, percentage, output=False):
    height, width = cv_image.shape[:2]

    # Create a progressive mask that only keeps the pixels, based on the percentage that this is indeed cylinder
    bottom_mask = np.zeros_like(mask)
    if percentage > 10:
        # Create a bottom mask for the lower 2 thirds of the image
        bottom_mask[int(height/3*2):, :] = 255
    else:
        # Create a bottom mask for the lower third of the image
        bottom_mask[int(height/3):, :] = 255
    #
    # bottom_mask[int(height/2):, :] = 255

    # Apply the bottom mask to the input mask using bitwise AND
    bottom_masked_mask = cv2.bitwise_and(mask, bottom_mask)

    # Check if any pixels in the masked image are non-zero
    if np.any(bottom_masked_mask != 0):
        if output:
            print("The mask is in the bottom half of the image.")
        return True
    else:
        if output:
            print("The mask is not in the bottom half of the image.")
        return False


class The_Ring:
    def __init__(self):
        rospy.init_node('clynder', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Publisher for the visualization markers
        self.markers_pub = rospy.Publisher('markers2', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # OUR ATTRIBUTES
        points_path = os.path.join(os.path.dirname(__file__), "newpoints.txt")
        self.path = read_path_log_orientation(points_path)
        self.path_idx = 0

        self.simple_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.cancel_goal_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)


    def get_pose(self,x_center,dist, time_stamp, marker_shape, marker_color, detected_object, detected_color):
        # parking_spaces are below rings - can share markers
        if detected_object == "parking_space":
            detected_object = "ring"
        if detected_color.lower() not in ("red", "green", "blue", "black", "yellow"):
            # skip 'unknown' color
            return

        # Calculate the position of the detected ellipse
        # k_f = 525 # kinect focal length in pixels
        k_f = 554 # kinect focal length in pixels

        elipse_x = self.dims[1] / 2 - x_center

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

        marker_coords = (point_world.point.x, point_world.point.y, point_world.point.z)
        ALL_MARKER_COORDS[detected_object][detected_color].append(marker_coords)

        # print(f"added {detected_color.upper()} marker for {detected_object}!")
        # print(f"Current markers for color {detected_color} and object {detected_object} are: {len(ALL_MARKER_COORDS[detected_object][detected_color])}")

        all_coordinates = np.array(ALL_MARKER_COORDS[detected_object][detected_color])
        avg_x = np.nanmean(all_coordinates[:, 0])
        avg_y = np.nanmean(all_coordinates[:, 1])
        avg_z = np.nanmean(all_coordinates[:, 2])

        # Create a Pose object with the same position
        pose = Pose()
        pose.position.x = avg_x
        pose.position.y = avg_y
        pose.position.z = avg_z
        
        if not dist:
            print(f"DIST: {dist}")
            print(marker_coords)
            print(f"avg_x: {avg_x}")
            print(f"avg_y: {avg_y}")
            print(f"avg_z: {avg_z}")
            print(f"detected_object: {detected_object}")
            print(f"detected_color: {detected_color}")
            return
        
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
        marker.scale = Vector3(0.2, 0.2, 0.2)
                # mybe we can place different markers for different objects (param in get pose or something)
        # so we can more 3easily destinguish them
        # same with different colors
        marker.color = marker_color


        
        BEST_MARKERS[detected_object][detected_color] = marker
        
        delete_arr = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = 'map'
        delete_arr.markers.append(delete_marker)
        self.markers_pub.publish(delete_arr)

        # self.marker_array = get_marker_array_to_publish()
        #
        markers_to_publish = get_marker_array_to_publish()
        self.markers_pub.publish(markers_to_publish)
        # print(f"PUBLISHED MARKER ARRAY OF LEN {len(markers_to_publish.markers)}!")


    def image_callback(self):
        try:
            data = rospy.wait_for_message('/camera/rgb/image_raw', Image)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            time_stamp_image_capture = rospy.Time.now()
            hsv_image =  cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)

        # closer might meen more in sync
        try:
            depth_img = rospy.wait_for_message('/camera/depth/image_raw', Image)
            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "16UC1")
            depth_time = depth_img.header.stamp
        except Exception as e:
            print(e)

        # Set the dimensions of the image
        self.dims = cv_image.shape

        # Tranform image to gayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ## make bottom two thirds  of image just be black
        # gray[self.dims[0]//3:,:] = 0


        cv_image_raw = cv_image.copy()


        has_color, color_mask, image_data = apply_colour_mask(cv_image, hsv_image)
        if has_color:
            # percentages = ", ".join([f"{color} ({image_data[color][1]}%)" for color in image_data])
            # print(f"Image contains colors: {percentages}")
            for color in image_data:
                percentage = image_data[color][1]
                # print(f"Color {color.upper()} ({percentage} %)")
                if is_mask_cylinder(cv_image, color_mask, percentage) and percentage > MINIMAL_ACCEPTED_COLOR_PERCENTAGE:
                    # find the contours of the binary color_mask
                    contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                    for contour in contours:
                        # Approximate the contour with a polygonal curve
                        epsilon = 0.1 * cv2.arcLength(contour, True)
                        approx = cv2.approxPolyDP(contour, epsilon, True)

                        # Check if the polygon has four sides and if the angles are close to 90 degrees
                        if len(approx) == 4:
                            angles = []
                            for i in range(4):
                                v1 = approx[(i+1)%4][0] - approx[i][0]
                                v2 = approx[i-1][0] - approx[i][0]
                                cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                                angle = np.degrees(np.arccos(cos_angle))
                                angles.append(angle)

                            area = cv2.contourArea(contour)
                            org = (200, 200)
                            font = cv2.FONT_HERSHEY_SIMPLEX
                            fontScale = 0.5
                            color_text = (0,255,0) if color != "green" else (255,255,255)
                            # cv2.putText(cv_image_raw, f"area: {area}", org, font, fontScale, color_text, 1)

                            if all(angle > 80 and angle < 100 for angle in angles) and area > MINIMAL_ACCEPTED_AREA:
                                M = cv2.moments(contour)

                                # Calculate the centroid of the contour
                                cx = int(M['m10']/M['m00'])
                                cy = int(M['m01']/M['m00'])

                                marker_colors = {
                                    "red": ColorRGBA(1,0,0,1),
                                    "green": ColorRGBA(0,1,0,1),
                                    "blue": ColorRGBA(0,0,1,1),
                                    "black": ColorRGBA(0,0,0,1),
                                    "unknown": ColorRGBA(0,0,0,1),
                                    "yellow": ColorRGBA(1,1,0,1),
                                }
                            
                                depth = depth_image[cy][cx]
                                # The contour is roughly rectangular
                                self.get_pose(cx,depth, depth_time, Marker.CYLINDER, marker_colors[color], detected_object="cylinder", detected_color=color)
                                cv2.drawContours(cv_image_raw, contour, -1, color_text, 2)

                                image_name = f"{dirs['cylinders'][color]}{color.upper()}_cylinder_{time.time()}.jpg"
                                # print(f"Found a {color.upper()} cylinder!")
                                cv2.imwrite(image_name, cv_image_raw)
                else:
                    # safety precaution - if it detects something above the middle of the image, don't skip
                    # DON'T DELETE!
                    break

        else:
            # print("Image does not contain cylinders!")
            # returns here - if no color, nothing interests us, skip
            return

def main():
    rospy.loginfo("clyinder_finder node started")
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
        
    rospy.sleep(1)

    while not rospy.is_shutdown():
        ring_finder.image_callback()
        rate.sleep()
    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
