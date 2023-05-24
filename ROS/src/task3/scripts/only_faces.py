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
from std_msgs.msg import ColorRGBA, String
from PIL import Image as _i
from tf.transformations import *
from sound_play.libsoundplay import SoundClient

from facenet_pytorch import InceptionResnetV1
import torch
from torchvision import transforms
import pytesseract
import easyocr

# OUR IMPORTS
import os
import math
from tf2_geometry_msgs import PoseStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
import shutil
from typing import List, Tuple
import time
from std_msgs.msg import Bool

import speech_recognition as sr
import nltk

nltk.download('punkt')
READER = easyocr.Reader(['en'])

DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

dirs = {
        "dir_faces" : os.path.join(os.path.dirname(__file__), "../last_run_info/faces/"),
        "dir_posters" : os.path.join(os.path.dirname(__file__), "../last_run_info/posters/"),
        }


GREET_POSITION_THRESHOLD = 3
MARKERS_NEEDED_FOR_GREETING = 6
NN_FACE_SIMILARITY_TOLERANCE = 0.75
NUMBER_OF_FACES_ON_THE_MAP = 3


class FaceRecogniser:
    def __init__(self):
        rospy.init_node('face_node', anonymous=True)
        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()
        
        # The function for performin HOG face detection
        protoPath = os.path.join(os.path.dirname(__file__), "./face_detector/deploy.prototxt.txt")
        modelPath = os.path.join(os.path.dirname(__file__), "./face_detector/res10_300x300_ssd_iter_140000.caffemodel")

        self.face_net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

        self.model = InceptionResnetV1(pretrained='vggface2')
        self.model.to(DEVICE)
        self.model.eval()
        for param in self.model.parameters():
            param.requires_grad = False
        self.transform = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)


        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher('markers_faces', MarkerArray, queue_size=1000)

        self.greet_pub = rospy.Publisher("/only_movement/greeting_is_going_on", Bool, queue_size=10)
        
        self.simple_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        self.twist_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)        

        # array containing face positions
        self.face_positions_and_images = []
        self.faces_dictionary = {}

        # This number is made up. The greatest measured distance is 4. The distance is probably in meters. 
        self.FACE_DISTANCE_TOLERANCE = 0.5

        self.faces = {}
        self.face_counter = 0
        self.posters = {}
        self.poster_counter = 0
        # all faces features
        # {
        #   "face1": {
        #       "ratio": _,
        #       "area": _,
        #       "all_features": [],
        #       "avg_feature": _,
        #       "all_positions": [],
        #       "avg_position": _,
        #       "all_greet_positions": [],
        #       "avg_greet_position": tuple(x,y,z),
        #   },
        #   "face2": {
        #       "ratio": _,
        #       "area": _,
        #       "all_features": [],
        #       "avg_feature": _,
        #       "all_positions": [],
        #       "avg_position": _,
        #       "all_greet_positions": [],
        #       "avg_greet_position": tuple(x,y,z),
        #   },
        # }

        # array of robot positions when faces are detected
        self.bot_positions = []

        self.traverse_sub = rospy.Subscriber("/only_movement/traversed_path/", Bool, self.traversed_callback)
        self.traversed_path = False

        self.robber_locations = set()
    
    def get_most_wanted_prison_color(self):
        most_wanted_color = None
        most_wanted_prize = 0
        for poster in self.posters:
            if self.posters[poster]["reward"] > most_wanted_prize:
                most_wanted_color = self.posters[poster]["color"]
                most_wanted_prize = self.posters[poster]["reward"]
        return most_wanted_color

    def traversed_callback(self, data):
        print("TRAVERSED CALLBACK (FACES)")
        
        if len(self.robber_locations) == 0:
            print("No robber locations found yet!")
        else:
            robber_cylinder_colors_pub = rospy.Publisher("/only_faces/robber_locations", String, queue_size=len(self.robber_locations))
            rospy.sleep(0.5)
            robber_cylinder = String()
            robber_cylinder.data = ",".join(list(self.robber_locations))
            robber_cylinder.data += f"\n{self.get_most_wanted_prison_color()}"
            robber_cylinder_colors_pub.publish(robber_cylinder)
            print("Published cylinder colors:", ','.join(list(self.robber_locations)))
        
        self.traversed_path = data.data
        

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
        for face_name, face_data in self.faces.items():
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
            # get me a orange color with rgba
            marker.color = ColorRGBA(1.0, 0.5, 0.0, 1.0)

            marker_array.markers.append(marker)
            count += 1

            ## GREETING POSITION MARKER
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
            # get me a orange color with rgba
            marker.color = ColorRGBA(1, 1, 1, 1.0)

            marker_array.markers.append(marker)
            count += 1


        
        self.markers_pub.publish(marker_array)


    def speak_msg(self, msg):
        """
        Function for speaking to a person.
        """
        soundhandle = SoundClient()
        voice = "voice_kal_diphone"
        volume = 1.
        rospy.loginfo(f"volume: {volume}, voice: {voice}, text:{msg}")
        soundhandle.say(msg, voice, volume)
    
    # ONLY CHECKING MARKER POSITION
    def get_closest_face(self, pose):
        closest_face_file_name = None
        closest_distance = float("inf")
        for file_name, face_data in self.faces.items():
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
    def extract_information(self, text): 
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
                if str_n[0] == "0":
                    str_n = "1" + str_n
                numbers.append(int(str_n))

        reward = max(numbers)

        text = text.lower()
        color = None
        # print(f"text: {text}")
        # do the same as the gree for colors red, black, blue, green
        if "green" in text:
            color = "green"
        elif "red" in text:
            color = "red"
        elif "black" in text:
            color = "black"
        elif "blue" in text:
            color = "blue"
        
        return reward, color

    def is_poster(self):
        poster_info = {}
        is_poster = False
        
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)

        text_pytesseract = pytesseract.image_to_string(rgb_image)
        text_easyocr = "\n".join([detection[1] for detection in READER.readtext(rgb_image)])

        # print("TEXT_PYTESSERACT", text_pytesseract)
        # print("TEXT_EASYOCR", text_easyocr)

        reward_pyt, color_pyt = self.extract_information(text_pytesseract)
        reward_easy, color_easy = self.extract_information(text_easyocr)
        
        rewards = [0]
        if reward_pyt:
            rewards.append(reward_pyt)
        if reward_easy:
            rewards.append(reward_easy)
        
        reward = max(rewards)
        
        color = color_pyt or color_easy
        if reward and color:
            print(f"REWARD: {reward}")
            print(f"COLOR: {color}")

            poster_info = {"reward": reward, "color": color}
            is_poster = True
            cv2.imwrite(os.path.join(dirs["dir_posters"], f"poster_{reward}_{color}_{self.poster_counter}.png"), rgb_image)
        elif not reward or not color:
            print("DID NOT EXTRACT ANY INFO")
            print("text_pytesseract:", text_pytesseract)

        return is_poster, poster_info

    def greet_person(self, greeting_pose):
        # PARKING NODE TAKES CONTROL OVER MOVEMENT
        greeting_no_movement = Bool()
        greeting_no_movement.data = True
        self.greet_pub.publish(greeting_no_movement)
        print("GREETING TAKING OVER")
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

        # Detect the faces in the image
        blob = cv2.dnn.blobFromImage(cv2.resize(rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)

        # no false positives yet, will keep as it is
        face_detections = self.face_net.forward()

        face_center = None
        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            if confidence>0.50:
                # rospy.loginfo("Saw new face.")
                box = face_detections[0,0,i,3:7] * np.array([w,h,w,h])
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
                face_center = (y1+(y2-y1)/2, x1+(x2-x1)/2)
                break
        
        if face_center is None:
            print("FACE CENTER IS NONE! CHECK!")
        else:
            point = (self.dims[0], self.dims[1]//2)   
            # print(f"face center: {face_center}")
            # print(f"point: {point}")

            angle_to_target = np.arctan2(face_center[0]-point[0], face_center[1]-point[1])
            angle_to_target = -(angle_to_target + np.pi/2)
            # print(f"angle to target: {angle_to_target}")
            # send twist msg to center the face 
            twist_rotation = Twist()
            twist_rotation.angular.z = angle_to_target
            twist_rotation.linear.x = 0.0
            self.twist_pub.publish(twist_rotation)
            print("centering")


            rospy.sleep(2)
            face_distance = float(np.nanmean(depth_image[y1:y2,x1:x2]))
            # send twist msg to move forward
            twist_forward = Twist()
            twist_forward.angular.z = 0.0
            twist_forward.linear.x = face_distance - 0.5
            print("moving forward")

            self.twist_pub.publish(twist_forward)
            rospy.sleep(1.5)

        poster_status, poster_info = self.is_poster()
        if not poster_status:
            ## SPEACH WITH THE PERSON
            self.speak_msg("Do you know where the robber is?")
            rospy.sleep(1)
            recognizer = sr.Recognizer()
            with sr.Microphone() as source:
                print()
                print()
                print("!!!SAY SOMETHING!!!")
                print()
                print()
                recognizer.adjust_for_ambient_noise(source)
                audio = recognizer.listen(source)
                print("recognizing")
                try:
                    text = recognizer.recognize_google(audio)
                    print("YOU SAID: ", text)
                    text = text.lower()
                    colors = ("blue", "green", "red", "yellow")
                    found_colors = set(text.split()).intersection(set(colors))
                    print(f"FOUND COLORS: {found_colors}")
                    self.robber_locations = self.robber_locations.union(found_colors)
                    # if "blue" in text:
                    #     self.robber_locations.append("blue")
                    # elif "red" in text:
                    #     self.robber_locations.append("red")
                    # elif "yellow" in text:
                    #     self.robber_locations.append("yellow")
                    # elif "green" in text:
                    #     self.robber_locations.append("green")
                    
                except sr.UnknownValueError:
                    print("Unable to recognize speech")
                except sr.RequestError as e:
                    print("Error:", str(e))

            
            self.speak_msg("Thank you for your help!")

            # just so we know what we heard)
            print(f"current robber positions: {self.robber_locations}")
        else:
            self.posters.update({self.poster_counter: poster_info})
            self.poster_counter += 1
            print("SAW A POSTER!")

        rospy.sleep(1)
        greeting_no_movement = Bool()
        greeting_no_movement.data = False
        self.greet_pub.publish(greeting_no_movement)

    def find_faces(self):

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

        # Detect the faces in the image
        blob = cv2.dnn.blobFromImage(cv2.resize(rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)

        # no false positives yet, will keep as it is
        face_detections = self.face_net.forward()


        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            if confidence>0.50:
                # rospy.loginfo("Saw new face.")
                box = face_detections[0,0,i,3:7] * np.array([w,h,w,h])
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
                
                ratio = abs(y2-y1)/abs(x2-x1)
                # print(f"Ratio is {ratio}")
                if  ratio > 2:
                    print(f"Skipping face. Not a nice image.")
                    return


                # Extract region containing face
                face_region = rgb_image[y1:y2, x1:x2]
                relative_path_to_face_region = os.path.join(dirs["dir_faces"], f"temp_face_region.jpg")
                image_name = os.path.join(dirs["dir_faces"], f"{time.time()}.jpg")
                try:
                    cv2.imwrite(relative_path_to_face_region, face_region) # Save the face image
                    
                    cv2.rectangle(rgb_image, (x1,y1), (x2,y2), (0,255,0), 2)

                    cv2.imwrite(image_name, rgb_image) # Save the face image
                except cv2.error:
                    # TODO: Assertion error, emtpy image
                    print(f"face_region is None. Not saving image to {relative_path_to_face_region}.")
                    continue

                # Find the distance to the detected face
                face_distance = float(np.nanmean(depth_image[y1:y2,x1:x2]))

                # Get the time that the depth image was recieved
                depth_time = depth_image_message.header.stamp

                # Find the location of the detected face
                pose = self.get_pose((x1,x2,y1,y2), face_distance, depth_time)

                if pose is not None:
                    closest_face = self.get_closest_face(pose)
                    greet_pose = self.get_greeting_pose(coords=(x1,x2,y1,y2), dist=face_distance, stamp=depth_time, pose_of_detection=p)
                    greeting_position = (greet_pose.position.x, greet_pose.position.y, greet_pose.position.z, greet_pose.orientation.z, greet_pose.orientation.w)
                    # we dont have any face close, (either empty or too far) NEW FACE
                    if not closest_face:

                        closest_face = f"face_{self.face_counter}"
                        face_data = {
                            "area": (x2-x1)*(y2-y1),
                            "ratio": ratio,
                            "all_positions": [(pose.position.x, pose.position.y, pose.position.z)],
                            "avg_position": (pose.position.x, pose.position.y, pose.position.z),
                            "all_features": [],
                            "avg_feature": None,
                            "all_greet_positions": [greeting_position],
                            "avg_greet_position": greeting_position,
                        }
                        self.faces[closest_face] = face_data
                        print(f"ADDED FACE : face_{self.face_counter}")
                        print(f"length: {len(self.faces)}")
                        self.face_counter += 1
                    else:
                        # we have a face close, add the new position
                        self.faces[closest_face]["all_positions"].append((pose.position.x, pose.position.y, pose.position.z))
                        self.faces[closest_face]["avg_position"] = np.mean(self.faces[closest_face]["all_positions"], axis=0)
                        self.faces[closest_face]["all_greet_positions"].append(greeting_position)
                        self.faces[closest_face]["avg_greet_position"] = np.mean(self.faces[closest_face]["all_greet_positions"], axis=0)

                        if len(self.faces[closest_face]["all_greet_positions"]) == GREET_POSITION_THRESHOLD:
                            # GO GREET PERSON
                            # center of the face
                            self.greet_person(self.faces[closest_face]["avg_greet_position"])
                
                self.update_markers()


def main():
    face_finder = FaceRecogniser()
    rospy.loginfo("Starting the face finder node")
    # this creates all the last run info folders

    
    rospy.sleep(2)
            
    while not rospy.is_shutdown():
        if not face_finder.traversed_path:
            face_finder.find_faces()
        else:
            rospy.sleep(5)
            rospy.loginfo("FACES: STOPPING NODE, WON'T NEED IT ANYMORE!")
            return
    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
