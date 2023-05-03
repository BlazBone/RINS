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
import subprocess
from exercise6_utils import read_path_log_orientation
from tf2_geometry_msgs import PoseStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
import shutil
from typing import List, Tuple
import time
from std_msgs.msg import Bool
from functools import reduce

dirs = {
        "dir_last_run_info" : os.path.join(os.path.dirname(__file__), f"../last_run_info/"),
        "dir_irregular" : os.path.join(os.path.dirname(__file__), "../last_run_info/irregular/"),
        "dir_cylinders" : os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/"),
        "dir_rings" : os.path.join(os.path.dirname(__file__), "../last_run_info/rings/"),
        "dir_park_spaces" : os.path.join(os.path.dirname(__file__), "../last_run_info/park_spaces_images/"),
        "cylinders": {
            "red": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/red/"),
            "blue": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/blue/"),
            "green": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/green/"),
            "yellow": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/yellow/"),
            },
        "rings": {
            "red": os.path.join(os.path.dirname(__file__), "../last_run_info/rings/red/"),
            "blue": os.path.join(os.path.dirname(__file__), "../last_run_info/rings/blue/"),
            "green": os.path.join(os.path.dirname(__file__), "../last_run_info/rings/green/"),
            "black": os.path.join(os.path.dirname(__file__), "../last_run_info/rings/black/"),
            },
        }

# calculated markers for all possible objects
ALL_MARKER_COORDS= {
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

class The_Ring:
    def __init__(self):
        rospy.init_node('ring_node', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Publisher for the visualization markers
        self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.simple_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.cancel_goal_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)

        self.park_pub = rospy.Publisher("/only_movement/park", Marker, queue_size=10)
        self.park_scanner_pub = rospy.Publisher("/only_movement/parking_search", Bool, queue_size=10)

        self.needs_to_be_parked = True

    def get_pose(self,e,dist, time_stamp, marker_shape, marker_color, detected_object, detected_color):
        # parking_spaces are below rings - can share markers
        if detected_object == "parking_space":
            detected_object = "ring"
        if detected_color.lower() not in ("red", "green", "blue", "black", "yellow"):
            # skip 'unknown' color
            return

        # Calculate the position of the detected ellipse
        # k_f = 525 # kinect focal length in pixels
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
        # take only the top half of the image
        gray = gray[:self.dims[0]//2,:]

        cv_image_raw = cv_image.copy()
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
            depth_time = depth_img.header.stamp
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



            # THIS SHOULD BE MOVED TO A BOOLEAN FUNCTION FOR CLARITY
            distance_to_background = abs(average_color_value - 118)
            distance_to_no_background = abs(average_color_value - 178)
            # MYBE COMBINE THIS two distance_to_background -  distance_to_no_background > treshold? 
            # cv2.imshow("cv_image", cv_image)
            # cv2.waitKey(1)
            ## distance difference mybe should be grater that 1 or 0.5 or something... 0 seems to be wrong since the ring can be tilted or the depth image is not rly accurate not sure 
            if distance_difference > 0 and min(inside_x_to_y, outside_x_to_y) > 0.75 and max(average_color) < 180 and min(average_color) > 100 and distance_to_background > distance_to_no_background:
                # print("Found a ring!")
                
                x, y = int(e2[0][0]), int(e2[0][1])
                w, h = int(e2[1][0] / 2), int(e2[1][1] / 2)
                # cut around elipse
                img = cv_image_raw[y-h:y+h, x-w:x+w]
                data = np.reshape(img, (-1,3))
    
                data = np.float32(data)

                if data.shape[0] <= 5:
                    # if somehow we get 0 
                    continue

                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
                flags = cv2.KMEANS_RANDOM_CENTERS
                _,_,centers = cv2.kmeans(data,5,None,criteria,10,flags)
               
                # TA SHIT PRAVILNO NRDI BARVE SAMO SO V RGB VALUES (HARD TO UNDERSTAND) THE JE FORA DA SE IZLOCIMO UNO SIVO IN UPORABIMO DRUGO SLIKO, KI NIMA NARISANIH ELIPS Z ZELENO.
                colors = centers[0:4]
                main_color = self.get_main_color(colors).lower()
                # print main color
                timestamp_img = time.time()

                marker_colors = {
                    "red": ColorRGBA(1,0,0,1),
                    "green": ColorRGBA(0,1,0,1),
                    "blue": ColorRGBA(0,0,1,1),
                    "black": ColorRGBA(0,0,0,1),
                    "unknown": ColorRGBA(0,0,0,1)
                }

                if main_color in ("red", "green", "blue", "black"):
                    image_name = f"{dirs['rings'][main_color]}{timestamp_img}.jpg"
                else:
                    image_name = f"{dirs['dir_irregular']}{timestamp_img}.jpg"
                
                if main_color == "green" and len(ALL_MARKER_COORDS["ring"]["green"]) >= 1 and self.needs_to_be_parked:
                    print("STARTED SCANNING FOR PARKING")
                    self.needs_to_be_parked = False

                    parking_scan_message = Bool()
                    parking_scan_message.data = True
                    self.park_scanner_pub.publish(parking_scan_message)

                    # park_message = BEST_MARKERS["ring"]["green"]
                    # self.park_pub.publish(park_message)

                    # print("READY TO BE PARKED, LISTEN TO /only_movement/park")

                

                cv2.line(cv_image, (x-w, 0), (x-w, cv_image.shape[0]), (0,0,255), 1)
                cv2.line(cv_image, (x+w, 0), (x+w, cv_image.shape[0]), (0,0,255), 1)

                cv2.line(cv_image, (0, y-h), (cv_image.shape[1], y-h), (255,0,0), 1)
                cv2.line(cv_image, (0, y+h), (cv_image.shape[1], y+h), (255,0,0), 1)
                
                # Create a masked array where zeros are masked
                depth_ring = depth_image[y-h:y+h, x-w:x+w]
                masked_a = np.ma.masked_equal(depth_ring, 0)
                # Compute the mean of the masked array
                mean = masked_a.mean()

                self.get_pose(e1, mean, depth_time, Marker.SPHERE, marker_colors[main_color], "ring", main_color)                

                cv2.imwrite(image_name, cv_image) # Save the whole image

            else:
                pass
                # image has no ring we skip it



    def get_main_color(self, colors: List):
        """
        recieves 4 most common colors in histogram (BGR ) values, and based on them it determines which color is dominant
        we can only have four
        RED, GREEN, BLUE, BLACK
        """
        ## here we just check first color in list 
        ## but we could easily extend function so that it checks all colors in list and returns list of colors
        for color in colors:
            b = color[0]
            g = color[1]
            r = color[2]
            # get max value of color
            max_color = max(b, g, r)
            #get all ratio of colors
            b_ratio = b / max_color
            g_ratio = g / max_color
            r_ratio = r / max_color
            # if red is dominant
            if r_ratio == 1 and g_ratio < 0.7 and b_ratio < 0.7:
                return "RED"
            # if green is dominant
            elif g_ratio == 1 and r_ratio < 0.7 and b_ratio < 0.7:
                return "GREEN"
            # if blue is dominant
            elif b_ratio == 1 and r_ratio < 0.5 and g_ratio < 0.8:
                return "BLUE"
            # if black is dominant
            
            if b < 80 and g < 80 and r < 80:
                return "BLACK"

        return "UNKNOWN"


def main():
    ring_finder = The_Ring()
    rospy.loginfo("Starting the ring finder node")
    for dirname, path in dirs.items():
        if "dir" in dirname:
            if os.path.exists(path):
                shutil.rmtree(path)
            os.mkdir(path)
        else:
            for color in dirs[dirname]:
                if os.path.exists(dirs[dirname][color]):
                    shutil.rmtree(dirs[dirname][color])
                os.mkdir(dirs[dirname][color])

    
    # rate = rospy.Rate(100)
    rospy.sleep(1)
            
    while not rospy.is_shutdown():
        ring_finder.image_callback()
        
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
