#!/usr/bin/python3

# DISABLE SHADOWS IN RINS_WORLD BEFORE STARTING THE PROGRAM - CLEARS THE IMAGE SIGNIFICANTLY
import rospy
import cv2
import numpy as np
## has to be for posestamped
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, PoseWithCovarianceStamped, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sound_play.libsoundplay import SoundClient
from typing import Tuple
import math
from tf.transformations import *
from actionlib_msgs.msg import GoalID, GoalStatusArray

import os
import shutil
from tf2_geometry_msgs import PoseStamped
from actionlib_msgs.msg import GoalID
import time

MARKER_COLORS = {
    "red": ColorRGBA(1,0,0,1),
    "green": ColorRGBA(0,1,0,1),
    "blue": ColorRGBA(0,0,1,1),
    "unknown": ColorRGBA(0,0,0,1),
    "yellow": ColorRGBA(1,1,0,1),
    "white": ColorRGBA(1,1,1,1),
}

# calculated markers for all possible objects
ALL_MARKER_COORDS= {
    "cylinder": {
        "red": list(),
        "blue": list(),
        "green": list(),
        "yellow": list(),
        }
    }

# best calculated markers for all possible objects
BEST_MARKERS= {
    "cylinder": {
        "red": None,
        "blue": None,
        "green": None,
        "yellow": None,
        }
    }

dirs = {

        "cylinders": {
            "red": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/red/"),
            "blue": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/blue/"),
            "green": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/green/"),
            "yellow": os.path.join(os.path.dirname(__file__), "../last_run_info/cylinders/yellow/"),
            },
        }


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


class The_Cylinder:
    def __init__(self):
        rospy.init_node('cylinder', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Publisher for the visualization markers
        self.markers_pub = rospy.Publisher('markers_cylinders', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        
        # of format:
        # color: cylinder_data
        self.cylinders = {}
        # OUR ATTRIBUTES

        self.simple_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        # self.cancel_goal_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
    
    def get_marker_array_to_publish(self):
        marker_array = MarkerArray()
        for color in self.cylinders:
            marker_array.markers.append(self.cylinders[color]["location"]) # cylinder marker
            marker_array.markers.append(self.cylinders[color]["greet_position"]) # greet position marker
            print(f"Added color '{color}' to published markers.")
        return marker_array

    def speak_msg(self, msg):
        """
        Function for speaking to a person.
        """
        soundhandle = SoundClient()
        rospy.sleep(0.1)

        voice = "voice_kal_diphone"
        volume = 1.
        rospy.loginfo(f"volume: {volume}, voice: {voice}, text:{msg}")
        soundhandle.say(msg, voice, volume)
    
    def coordinates_to_pose(self, coords):
        """
        Transforms tuple (x, y, z) to Pose.
        """
        try:
            x,y,z = coords
        except ValueError:
            x,y,z,_,_ = coords
        except:
            raise ValueError(f"Coordinates are of len {len(coords)}.\nValues are {coords}.")

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.z = 1
        pose.orientation.w = 0
        pose.orientation.x = 0
        pose.orientation.y = 0

        return pose
    
    def pose_to_marker(self, pose, color: str, size: float=0.2):
        """
        Transforms Pose to Marker.
        """
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.pose = pose
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration(1000) # this way marker stays up until deleted
        marker.id = self.marker_num
        self.marker_num += 1
        marker.scale = Vector3(size, size, size)
        marker.color = MARKER_COLORS[color]
        return marker


    def create_markers(self, detected_color: str):
        """
        Returns a tuple (marker_cylinder, marker_greet).
        """
        location_marker = None
        greet_marker = None

        if self.cylinders.get(detected_color):
            all_locations = self.cylinders[detected_color]["all_locations"]
            all_locations = np.array(all_locations)
            location = np.mean(all_locations, axis=0)[:3]
            print("LOCATION", location)
            location_pose = self.coordinates_to_pose(location)
            location_marker = self.pose_to_marker(location_pose, color=detected_color)
            print("LOCATION MARKER", location_marker)

            all_greet_positions = self.cylinders[detected_color]["all_greet_positions"]
            all_greet_positions = np.array(all_greet_positions)
            greet_position = np.mean(all_greet_positions, axis=0)[:3]
            print("GREET POSITION", greet_position)
            greet_pose = self.coordinates_to_pose(greet_position)
            greet_marker = self.pose_to_marker(greet_pose, color="white", size=0.1)
            print("GREET MARKER", greet_marker)
            
            if location_marker:
                self.cylinders[detected_color]["location"] = location_marker
                print(f"ADDED LOCATION MARKER: {location_marker.pose.position}")
            else:
                print("LOCATION MARKER NONE")
            if greet_marker:
                self.cylinders[detected_color]["greet_position"] = greet_marker
                print(f"ADDED GREET: {greet_marker.pose.position}")
            else:
                print("GREET MARKER NONE")


    def delete_markers(self):
        delete_arr = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = 'map'
        delete_arr.markers.append(delete_marker)
        self.markers_pub.publish(delete_arr)

    def get_pose(self,x_center,dist, time_stamp, marker_color, detected_object, detected_color):
        # parking_spaces are below rings - can share markers
        if detected_object == "parking_space":
            raise ValueError("parking_space is not a valid object for this function")
        if detected_color.lower() not in ("red", "green", "blue", "yellow"):
            # skip 'unknown' color
            raise ValueError(f"{detected_color} is not a valid color for this function")

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

        pose = Pose()
        pose.position.x = point_world.point.x
        pose.position.y = point_world.point.y
        pose.position.z = point_world.point.z

        # marker_coords = (point_world.point.x, point_world.point.y, point_world.point.z)
        # # ALL_MARKER_COORDS[detected_object][detected_color].append(marker_coords)
        # self.cylinders[detected_color]["all_positions"].append(marker_coords)

        # print(f"added {detected_color.upper()} marker for {detected_object}!")
        # print(f"Current markers for color {detected_color} and object {detected_object} are: {len(ALL_MARKER_COORDS[detected_object][detected_color])}")

        # all_coordinates = np.array(ALL_MARKER_COORDS[detected_object][detected_color])
        # all_coordinates = np.array(self.cylinders[detected_color]["all_positions"])

        # avg_x = np.nanmean(all_coordinates[:, 0])
        # avg_y = np.nanmean(all_coordinates[:, 1])
        # avg_z = np.nanmean(all_coordinates[:, 2])

        # Create a Pose object with the same position
        # avg_pose = Pose()
        # avg_pose.position.x = avg_x
        # avg_pose.position.y = avg_y
        # avg_pose.position.z = avg_z
        
        if not dist:
            print(f"DIST: {dist}")
            # print(marker_coords)
            # print(f"avg_x: {avg_x}")
            # print(f"avg_y: {avg_y}")
            # print(f"avg_z: {avg_z}")
            # print(f"detected_object: {detected_object}")
            # print(f"detected_color: {detected_color}")
            # return
        
        # so we get no errors
        # avg_pose.orientation.z = 1
        # avg_pose.orientation.w = 0
        # avg_pose.orientation.x = 0
        # avg_pose.orientation.y = 0
        # 
        # PUBLISHING MARKER
        # Create a marker used for visualization
        # self.marker_num += 1
        # marker = Marker()
        # marker.header.stamp = point_world.header.stamp
        # marker.header.frame_id = point_world.header.frame_id
        # marker.pose = avg_pose
        # marker.type = Marker.CYLINDER
        # marker.action = Marker.ADD
        # marker.frame_locked = False
        # marker.lifetime = rospy.Duration(1000) # this way marker stays up until deleted
        # marker.id = self.marker_num
        # marker.scale = Vector3(0.2, 0.2, 0.2)
        # marker.color = marker_color


        # BEST_MARKERS[detected_object][detected_color] = marker
        # self.cylinders[detected_color]["avg_position"] = marker

        # detected color should already be in self.cylinders
        # if self.cylinders.get(detected_color):
        #     self.create_markers(detected_color=detected_color)
        #     print("CREATED MARKERS")

        # markers_to_publish = self.get_marker_array_to_publish()
        # self.markers_pub.publish(markers_to_publish)

        return pose

    def get_greeting_pose(self, coords: float, dist : float, stamp, pose_of_detection: PoseWithCovarianceStamped) -> Pose:
        """
        Calculates the greeting position for the bot to travel to.
        """

        # Calculate the position of the detected face
        k_f = 554 # kinect focal length in pixels

        face_x = self.dims[1] / 2 - coords

        angle_to_target = np.arctan2(face_x,k_f)

        # Get the angles in the base_link relative coordinate system
        # step away from image 0.4 units (probably meters)
        dist -= 0.4
        x, y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)


        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        # Define a quaternion for the rotation
        # Roatation is such that the image of the face is in the center of the robot view.
        q = Quaternion()
        q.x = 0
        q.y = 0
        q.z = math.sin(angle_to_target)
        q.w = math.cos(angle_to_target)

    
        q2 = pose_of_detection.pose.pose.orientation

        goal_quaternion = quaternion_multiply((q2.x, q2.y, q2.z, q2.w), (q.x, q.y, q.z, q.w))

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")

            # Create a Pose object with the same position
            pose = Pose()

            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z

            pose.orientation.x = goal_quaternion[0]
            pose.orientation.y = goal_quaternion[1]
            pose.orientation.z = goal_quaternion[2]
            pose.orientation.w = goal_quaternion[3]
            
        except Exception as e:
            print(e)
            pose = None

        return pose
    
    def go_to_greet(self, greeting_pose):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time().now()
        msg.pose.position.x = greeting_pose[0]
        msg.pose.position.y = greeting_pose[1]
        msg.pose.orientation.z = greeting_pose[3]
        msg.pose.orientation.w = greeting_pose[4]
        
        print("PUBLISHING GREETING POSITION")
        self.simple_goal_pub.publish(msg)
        
        while True:
            status = rospy.wait_for_message("/move_base/status", GoalStatusArray)
            if status.status_list[-1].status not in (0, 1):
                print("REACHED GREETING POSE")
                break

        rospy.sleep(1)
    def image_callback(self):
        try:
            data = rospy.wait_for_message('/camera/rgb/image_raw', Image)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv_image =  cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            depth_img = rospy.wait_for_message('/camera/depth/image_raw', Image)
            depth_time = depth_img.header.stamp
            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "16UC1")
            depth_time = depth_img.header.stamp
            p = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = cv_image.shape

        cv_image_raw = cv_image.copy()

        has_color, color_mask, image_data = apply_colour_mask(cv_image, hsv_image)
        if has_color:
            for color in image_data:
                percentage = image_data[color][1]
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
                            color_text = (0,255,0) if color != "green" else (255,255,255)

                            if all(angle > 80 and angle < 100 for angle in angles) and area > MINIMAL_ACCEPTED_AREA:
                                M = cv2.moments(contour)

                                # Calculate the centroid of the contour
                                cx = int(M['m10']/M['m00'])
                                cy = int(M['m01']/M['m00'])

                                marker_colors = {
                                    "red": ColorRGBA(1,0,0,1),
                                    "green": ColorRGBA(0,1,0,1),
                                    "blue": ColorRGBA(0,0,1,1),
                                    "unknown": ColorRGBA(0,0,0,1),
                                    "yellow": ColorRGBA(1,1,0,1),
                                }
                            
                                image_name = f"{dirs['cylinders'][color]}{color.upper()}_cylinder_{time.time()}.jpg"
                                print(f"Found a {color.upper()} cylinder!")
                                if len(ALL_MARKER_COORDS["cylinder"][color]) == 0:
                                    self.speak_msg(f"{color} cylinder")

                                # The contour is roughly rectangular
                                depth = depth_image[cy][cx]
                                cylinder_pose = self.get_pose(cx,depth, depth_time, marker_colors[color], detected_object="cylinder", detected_color=color)
                                cylinder_location = (cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z)
                                cv2.drawContours(cv_image_raw, contour, -1, color_text, 2)
                                cv2.imwrite(image_name, cv_image_raw)

                                greet_pose = self.get_greeting_pose(coords=cx, dist=depth, stamp=depth_time, pose_of_detection=p)
                                greeting_position = (greet_pose.position.x, greet_pose.position.y, greet_pose.position.z, greet_pose.orientation.z, greet_pose.orientation.w)

                                # we dont have any face close, (either empty or too far) NEW CYLINDER
                                if not self.cylinders.get(color):
                                    cylinder_data = {
                                        "all_locations": [(greet_pose.position.x, greet_pose.position.y, greet_pose.position.z)],
                                        "location": None,
                                        "all_greet_positions": [greeting_position],
                                        "greet_position": None,
                                    }
                                    self.cylinders.update({color: cylinder_data})
                                else:
                                    # we have a cylinder close, add the new position
                                    self.cylinders[color]["all_locations"].append(cylinder_location)
                                    self.cylinders[color]["all_greet_positions"].append(greeting_position)

                                self.create_markers(detected_color=color)
                                self.delete_markers()

                                markers_to_publish = self.get_marker_array_to_publish()
                                print("Markers to publish", markers_to_publish)
                                print(markers_to_publish)
                                self.markers_pub.publish(markers_to_publish)


                else:
                    # safety precaution - if it detects something above the middle of the image, don't skip
                    # DON'T DELETE!
                    break

        else:
            # print("Image does not contain cylinders!")
            # returns here - if no color, nothing interests us, skip
            return

def main():
    ring_finder = The_Cylinder()
    
    rospy.loginfo("Starting the cylinder node")
    rate = rospy.Rate(10)
        
    rospy.sleep(1)

    while not rospy.is_shutdown():
        ring_finder.image_callback()
        rate.sleep()
    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
