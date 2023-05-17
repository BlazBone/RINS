#!/usr/bin/python3

# DISABLE SHADOWS IN RINS_WORLD BEFORE STARTING THE PROGRAM - CLEARS THE IMAGE SIGNIFICANTLY

import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, Quaternion, PoseWithCovarianceStamped, Twist
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from PIL import Image as _i
from tf.transformations import *
from sound_play.libsoundplay import SoundClient





# OUR IMPORTS
import pytesseract
import os
import math
from tf2_geometry_msgs import PoseStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
import shutil
from typing import List, Tuple
import time
from std_msgs.msg import Bool


dirs = {
        "dir_posters" : os.path.join(os.path.dirname(__file__), "../last_run_info/posters/"),
        }


GREET_POSITION_THRESHOLD = 3
MARKERS_NEEDED_FOR_GREETING = 6
NN_FACE_SIMILARITY_TOLERANCE = 0.75
NUMBER_OF_FACES_ON_THE_MAP = 3


class PosterRecognizer:
    def __init__(self):
        rospy.init_node('face_node', anonymous=True)
        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()
        
        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)


        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher('markers_posters', MarkerArray, queue_size=1000)

        self.greet_pub = rospy.Publisher("/only_movement/greeting_is_going_on", Bool, queue_size=10)
        
        self.simple_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        self.twist_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)        

        # array containing face positions
        self.poster_positions_and_images = []
        self.poster_dictionary = {}

        # This number is made up. The greatest measured distance is 4. The distance is probably in meters. 
        self.FACE_DISTANCE_TOLERANCE = 0.5

        self.posters = {}
        self.poster_counter = 0
        # all faces features
        # {
        #   "face1": {
        #       "reward": _,
        #      "prision_color": _,
        #       "all_positions": [],
        #       "avg_position": _,
        #       "all_greet_positions": [],
        #       "avg_greet_position": tuple(x,y,z),
        #   },
        #   "face2": {
        #       "reward": _,
        #      "prision_color": _,
        #       "all_positions": [],
        #       "avg_position": _,
        #       "all_greet_positions": [],
        #       "avg_greet_position": tuple(x,y,z),
        #   },
        # }
        
    def get_greeting_pose(self, coords : Tuple[int,int,int,int], dist : float, stamp, pose_of_detection: PoseWithCovarianceStamped) -> Pose:
        """
        Calculates the greeting position for the bot to travel to.
        """

        # Calculate the position of the detected face
        k_f = 554 # kinect focal length in pixels

        x1, x2, _, _ = coords

        face_x = self.dims[1] / 2 - (x1+x2)/2.

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
    
    def update_markers(self):
        #remove all markers
        delete_arr = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = 'map'
        delete_arr.markers.append(delete_marker)
        self.markers_pub.publish(delete_arr)

        #publish new markers
        marker_array = MarkerArray()
        count = 0
        for face_name, face_data in self.posters.items():
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"

            marker.pose.position.x = face_data["avg_position"][0]
            marker.pose.position.y = face_data["avg_position"][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.orientation.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Duration(1000) # this way marker stays up until deleted
            marker.id = count
            marker.scale = Vector3(0.2, 0.2, 0.2)
            # get me a bright pink color
            marker.color = ColorRGBA(1, 0, 1, 1.0)

            marker_array.markers.append(marker)
            count += 1

            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"

            marker.pose.position.x = face_data["avg_greet_position"][0]
            marker.pose.position.y = face_data["avg_greet_position"][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
        
            marker.pose.orientation.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Duration(1000) # this way marker stays up until deleted
            marker.id = count
            marker.scale = Vector3(0.1, 0.1, 0.1)
            marker.color = ColorRGBA(1, 1, 1, 1.0)
            count += 1

            marker_array.markers.append(marker)


        
        self.markers_pub.publish(marker_array)


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
    
    # ONLY CHECKING MARKER POSITION
    def get_closest_face(self, pose):
        closest_face_file_name = None
        closest_distance = float("inf")
        for file_name, face_data in self.posters.items():
            avg_position = face_data["avg_position"]
            dist = np.sqrt((pose.position.x - avg_position[0])**2 + (pose.position.y - avg_position[1])**2)
            if dist < closest_distance:
                closest_distance = dist
                closest_face_file_name = file_name
        
        # closest face is too far (facedetected is probably new)
        if  closest_distance > self.FACE_DISTANCE_TOLERANCE:
            return None
        
        return closest_face_file_name



    def get_pose(self,coords,dist,stamp):
        # Calculate the position of the detected face

        k_f = 554 # kinect focal length in pixels

        x1, x2, y1, y2 = coords

        face_x = self.dims[1] / 2 - (x1+x2)/2.
        face_y = self.dims[0] / 2 - (y1+y2)/2.

        angle_to_target = np.arctan2(face_x,k_f)

        # Get the angles in the base_link relative coordinate system
        x, y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")

            # Create a Pose object with the same position
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z
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

    def greet_person(self, greeting_pose, closest_poster):
        # PARKING NODE TAKES CONTROL OVER MOVEMENT
        greeting_no_movement = Bool()
        greeting_no_movement.data = True
        self.greet_pub.publish(greeting_no_movement)
        print("GREETING POSTER TAKING OVER")
        rospy.sleep(1)
        
        self.go_to_greet(greeting_pose)
            
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
            depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
            p = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        except Exception as e:
            print(e)
            return 0

        # Convert the images into a OpenCV (numpy) format

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]

        # again detect letters
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        data = pytesseract.image_to_data(gray_image, output_type=pytesseract.Output.DICT)

        centers_of_poster = []
        # Iterate over the detected regions choose wanted and save the middle of the wanted word location
        for i in range(len(data["text"])):
            text = data["text"][i]
            x, y, w, h = data["left"][i], data["top"][i], data["width"][i], data["height"][i]
            center_of_poster = (y+(h-y)/2, x+(w-x)/2)
            centers_of_poster.append(center_of_poster)
        
        # get the center of the poster
        center_of_poster = None
        if centers_of_poster:
            center_of_poster = np.mean(centers_of_poster, axis=0)
        else:
            print("NO POSTER CENTER FOUND")
            return

        point = (self.dims[0], self.dims[1]//2)   
        print(f"face center: {center_of_poster}")
        print(f"point: {point}")
        angle_to_target = np.arctan2(center_of_poster[0]-point[0], center_of_poster[1]-point[1])
        angle_to_target = -(angle_to_target + np.pi/2)
        print(f"angle to target: {angle_to_target}")
        # send twist msg to center the face 
        twist_rotation = Twist()
        twist_rotation.angular.z = angle_to_target
        twist_rotation.linear.x = 0.0
        self.twist_pub.publish(twist_rotation)
        print("centering")


        rospy.sleep(2)
        face_distance = float(np.nanmean(depth_image[int(center_of_poster[0]-10):int(center_of_poster[0]+10),int(center_of_poster[1]-10):int(center_of_poster[1]+10)]))
        print(f"face distance: {face_distance}")
        # send twist msg to move forward
        twist_forward = Twist()
        twist_forward.angular.z = 0.0
        twist_forward.linear.x = face_distance - 0.4
        print("moving forward")

        rospy.sleep(2)

        self.twist_pub.publish(twist_forward)

        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except Exception as e:
            print(e)
            return 0

        # again detect letters
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        text = pytesseract.image_to_string(gray_image)
        
        number_text = text.replace("o", "0")
        # GET THE REWARD
        lines_info = [line.replace(" ", "") for line in number_text.split("\n")]
        # remove empy lines
        lines_info = list(filter(lambda x: x != "", lines_info))

        numbers = [0]
        for line in lines_info:
            #filter out only the numbers
            str_n = "".join(filter(str.isdigit, line))

            if str_n != "":
                numbers.append(int(str_n))
        reward = max(numbers)
        # set reward
        self.posters[closest_poster]["reward"] = reward


        text = text.lower()
        print(f"text: {text}")
        # do the same as the gree for colors red, black, blue, green
        if "green" in text:
            self.posters[closest_poster]["prison"] = "green"
        elif "red" in text:
            self.posters[closest_poster]["prison"] = "red"
        elif "black" in text:
            self.posters[closest_poster]["prison"] = "black"
        elif "blue" in text:
            self.posters[closest_poster]["prison"] = "blue"
        else:
            self.posters[closest_poster]["prison"] = "none"

        # save the image
        cv2.imwrite(os.path.join(dirs["dir_posters"], f"{closest_poster}_closeup.jpg"), rgb_image)
        print(f"found robber with reward: {reward} and prison color: {self.posters[closest_poster]['prison']}")

        rospy.sleep(2)

        greeting_no_movement = Bool()
        greeting_no_movement.data = False
        self.greet_pub.publish(greeting_no_movement)

    def find_posters(self):
        # Get the next rgb and depth images that are posted from the camera
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
            depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
            p = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        except Exception as e:
            print(e)
            return 0

        # Convert the images into a OpenCV (numpy) format

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]


                # Preprocess the image (if necessary)
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Perform OCR using Tesseract
        # Pass any additional configurations as needed
        data = pytesseract.image_to_data(gray_image, output_type=pytesseract.Output.DICT)

        # Iterate over the detected regions
        for i in range(len(data["text"])):
            text = data["text"][i]
            x, y, w, h = data["left"][i], data["top"][i], data["width"][i], data["height"][i]
            
            # Filter text based on specific criteria
            if "WANTED" in text:
                # we spotted a poster
                # save it to the posters folder
                # save the image     
                # Display the text and its coordinates
                print("Text:", text)
                print("Coordinates: x={}, y={}, width={}, height={}".format(x, y, w, h))
                print("--------------------")
                poster_distance = float(np.nanmean(depth_image[y:y+h,x:x+w]))

                deph_time = depth_image_message.header.stamp

                pose = self.get_pose((x,x+w,y,y+h), poster_distance, deph_time)

                if pose is not None:
                    closest_poster = self.get_closest_face(pose)
                    greet_pose = self.get_greeting_pose(coords=(x,x+w,y,y+h), dist=poster_distance, stamp=deph_time, pose_of_detection=p)
                    greeting_position = (greet_pose.position.x, greet_pose.position.y, greet_pose.position.z, greet_pose.orientation.z, greet_pose.orientation.w)
                    # we dont have any face close, (either empty or too far) NEW FACE
                    if not closest_poster:
                        closest_poster = f"poster_{self.poster_counter}"
                        poster_data = {
                            "reward": 0,
                            "prison_color": "",
                            "all_positions": [(pose.position.x, pose.position.y, pose.position.z)],
                            "avg_position": (pose.position.x, pose.position.y, pose.position.z),
                            "all_greet_positions": [greeting_position],
                            "avg_greet_position": greeting_position,
                        }
                        cv2.imwrite(os.path.join(dirs["dir_posters"], f"{closest_poster}_first.jpg"), rgb_image)   
                        self.posters[closest_poster] = poster_data
                        print(f"ADDED POSTER : poster_{self.poster_counter}")
                        print(f"length: {len(self.posters)}")
                        print(f"posters: {self.posters}")
                        self.poster_counter += 1
                    else:
                        # we have a face close, add the new position
                        self.posters[closest_poster]["all_positions"].append((pose.position.x, pose.position.y, pose.position.z))
                        self.posters[closest_poster]["avg_position"] = np.mean(self.posters[closest_poster]["all_positions"], axis=0)
                        self.posters[closest_poster]["all_greet_positions"].append(greeting_position)
                        self.posters[closest_poster]["avg_greet_position"] = np.mean(self.posters[closest_poster]["all_greet_positions"], axis=0)
                        
                        if len(self.posters[closest_poster]["all_greet_positions"]) == GREET_POSITION_THRESHOLD:
                            # GO GREET PERSON
                            # center of the face
                            self.greet_person(self.posters[closest_poster]["avg_greet_position"], closest_poster)

                self.update_markers()

def main():
    poster_recognizer = PosterRecognizer()
    rospy.loginfo("Starting the POSTER finder node")
    # this creates all the last run info folders

    rospy.sleep(2)
            
    while not rospy.is_shutdown():
        poster_recognizer.find_posters()
    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
