#!/usr/bin/env python
"""
Code for recognising AprilTags in a room and moving through a selected doorwayss
"""

# Imports
##########################
import os
import sys
from math import radians  # This is used to reset the head pose
import math
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library
import apriltag
from itertools import chain
import collections
import camera_configs
from apriltag_perception import AprilTagPerception

import rospy  # ROS Python interface
from sensor_msgs.msg import CompressedImage  # ROS CompressedImage message
from sensor_msgs.msg import JointState  # ROS joints state message
from cv_bridge import CvBridge, CvBridgeError  # ROS -> OpenCV converter
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control) message
import random
import miro2 as miro  # Import MiRo Developer Kit library

try:
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2
##########################


class MiRoClient:
    """
    Script settings below
    """
    TICK = 0.01  # Update interval for main control loop in secs
    CAM_FREQ = 1  # Number of ticks before camera gets a new frame, increase in case of network lag
    NODE_EXISTS = False  # Disables (True) / Enables (False) rospy.init_node
    SLOW = 0.05  # Radial speed when turning on the spot (rad/s)
    MEDIUM = 0.3 # Linear speed
    FAST = 0.7  # Linear speed
    DEBUG = True # Set to True to enable debug views of the cameras
    SONAR_THRESH = 0.12

    def reset_head_pose(self):
        """
        Reset MiRo head to default position, to avoid having to deal with tilted frames
        """
        self.kin_joints = JointState()  # Prepare the empty message
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, radians(34.0), 0.0, 0.0]
        t = 0
        while not rospy.core.is_shutdown():  # Check ROS is running
            # Publish state to neck servos for 1 sec
            self.pub_kin.publish(self.kin_joints)
            rospy.sleep(self.TICK)
            t += self.TICK
            if t > 1:
                break

    def drive(self, speed_l=0.1, speed_r=0.1):  # (m/sec, m/sec)
        """
        Wrapper to simplify driving MiRo by converting wheel speeds to cmd_vel
        """
        # Prepare an empty velocity command message
        msg_cmd_vel = TwistStamped()

        # Desired wheel speed (m/sec)
        wheel_speed = [speed_l, speed_r]

        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

        # Update the message with the desired speed
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # Publish message to control/cmd_vel topic
        self.vel_pub.publish(msg_cmd_vel)

    def callback_caml(self, ros_image):  # Left camera
        self.callback_cam(ros_image, 0)

    def callback_camr(self, ros_image):  # Right camera
        self.callback_cam(ros_image, 1)

    def callback_cam(self, ros_image, index):
        """
        Callback function executed upon image arrival
        """
        # Silently(-ish) handle corrupted JPEG frames
        try:
            # Convert compressed ROS image to raw CV image
            image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            # Convert from OpenCV's default BGR to RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # Store image as class attribute for further use
            self.input_camera[index] = image
            # Get image dimensions
            self.frame_height, self.frame_width, channels = image.shape
            self.x_centre = self.frame_width / 2.0
            self.y_centre = self.frame_height / 2.0
            # Raise the flag: A new frame is available for processing
            self.new_frame[index] = True
        except CvBridgeError as e:
            # Ignore corrupted frames
            pass

    def callback_package(self, msg):
        self.sonar = msg.sonar.range


    # Flattens retrived images from cameras to fix radial distortion
    def rectifyImages(self, image, index):
        if index == 0:
            img_rectified = cv2.remap(image, \
                            camera_configs.left_map1, \
                            camera_configs.left_map2, \
                            cv2.INTER_LINEAR)
        else:
            img_rectified = cv2.remap(image, \
                            camera_configs.right_map1, \
                            camera_configs.right_map2, \
                            cv2.INTER_LINEAR)
        return img_rectified


    ## MAIN FUNCIONS ##
    def detect_AprilTags(self, frame, index):
        tag = None
        if frame is None: # Sanity check
            return

        # Debug window to show the frame
        if self.DEBUG:
            cv2.imshow("camera" + str(index), frame)
            cv2.waitKey(1)

        # Flag this frame as processed, so that it's not reused in case of lag
        self.new_frame[index] = False
        tags = self.tag.detect_tags(frame)
        centres = []
        tagsFound = []

        if tags:
            for i in range (len(tags)):
                tagsFound.append([tags[i].id, tags[i].distance, tags[i].centre])
	return tagsFound

    # First tag indicates room MiRo is in - then looks for rest of AprilTags defined to be in given room
    # Switches to move through doorway once all AprilTags in a given room are identified
    def look_for_AprilTags(self):
        """
        [1 of 3] Wander MiRo if it doesn't see an AprilTag in its current
        position, until it sees one.
        """
        if len(self.foundRoomTags) == 4:
            self.status_code=4
            self.mapping_done = True
            print('Mapping is complete')
            return

        if self.just_switched:  # Print once
            print("MiRo is looking for the AprilTags in the room...")
            self.just_switched = False

        if self.new_frame[0]:
            image = self.input_camera[0]
            flat_img = self.rectifyImages(image, 0)
            # Run the detect AprilTag procedure
            detected = self.detect_AprilTags(flat_img, 0)
            detected = list(filter(lambda x: x[0]<13, detected))

            if len(detected) > 0 :
                for key in self.dictAllTags:
                    vals = self.dictAllTags.get(key) # get the values for a given key (room name)
                    flat = list(chain.from_iterable(vals)) # flatten key values (roomt ags)
                    if detected[0][0] in flat:
                        self.currentRoom = key
                        print('curr', self.currentRoom)
                        self.allRoomTags = flat

                        break

            for tag in detected:
                if tag[0] not in [item[0] for item in self.foundRoomTags] and tag[0] in self.allRoomTags:


                    self.foundRoomTags.append(tag)
                    self.tags_used.append(tag)
                    print('found', self.foundRoomTags)
                    print('used', self.tags_used)
        # Have all AprilTags been detected?
        justIdsFound = [item[0] for item in self.tags_used]
        if sorted(justIdsFound) == sorted(self.allRoomTags) and len(self.allRoomTags) > 0:
            print("Found all tags in room: ", justIdsFound)
            self.status_code = 2
            self.just_switched = True
        else:
            self.drive(self.SLOW, -self.SLOW)


    def find_Doorway(self):
        """
        [2 of 3] Once all AprilTags in the room are found and the room it is in is known...
        Detect a doorway to move through
        """

        if self.just_switched:  # Print once
            print("MiRo is finding a doorway...")
            self.just_switched = False

        for index in range(2):  # For each camera (0 = left, 1 = right)
            # Skip if there's no new image, in case the network is choking
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            flat_img = self.rectifyImages(image, index)
            # Run the detect ball procedure
            self.doorway[index] = self.detect_AprilTags(flat_img, index)

        for i, y in enumerate(self.doorway):
            self.doorway[i] = list(filter(lambda x:  x[0] < 13, self.doorway[i])) if y else y


        if not self.doorway[0] and not self.doorway[1]:
            if self.middle_flag:
                self.drive(0.4, 0.4)
                self.doorway = [None,None]
                self.status_code = 3
                self.just_switched = True
                self.middle_flag = False
            else:
                self.drive(0.2, 0.4)
        elif self.doorway[0] and self.doorway[1]:
            if len(self.doorway[0]) == 1 and len(self.doorway[1]) == 1:
                self.middle_flag=True
                self.drive(0.25, 0.25)
            elif len(self.doorway[0]) == 2 and len(self.doorway[1]) < 2:
                self.middle_flag = True
                self.drive(0.2, 0.4)
            elif len(self.doorway[0]) < 2 and len(self.doorway[1]) == 2:
                self.middle_flag = True
                self.drive(0.4, 0.2)


        elif not self.doorway[0] and self.doorway[1]:
            tags_in_sight = [x[0] for x in self.doorway[1]]
            even_tag = list(filter( lambda x: x%2 == 0, tags_in_sight))
            if even_tag:
                self.drive(0.4, 0.2)
            else:
                self.drive(0.2, 0.4)
        elif self.doorway[0] and not self.doorway[1]:
            tags_in_sight = [x[0] for x in self.doorway[0]]
            odd_tag = list(filter( lambda x: x%2 == 1, tags_in_sight))
            if odd_tag:
                self.drive(0.2, 0.4)
            else:
                self.drive(0.4, 0.2)
        else:
            print(5)
            #print(len(self.doorway[0]), len(self.doorway[1]))
            self.drive(-self.SLOW, self.SLOW)


    def lock_onto_target(self, error=25):
        """
        [2 of 3] Once a ball has been detected, turn MiRo to face it
        """
        if self.just_switched:  # Print once
            print("MiRo is Heading into the new Room")
            self.just_switched = False

        for index in range(2):  # For each camera (0 = left, 1 = right)
            # Skip if there's no new image, in case the network is choking
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            flat_img = self.rectifyImages(image, index)
            # Run the detect ball procedure
            self.target[index] = self.detect_AprilTags(flat_img, index)

        for i, y in enumerate(self.target):
            self.target[i] = list(filter(lambda x:  x[0] >= 13, self.target[i])) if y else y

        target_distances = []
        if not self.target[0] and not self.target[1]:
            self.drive(self.SLOW, self.SLOW)
        else:

            for i, y in enumerate(self.target):
                target_distances.extend([x[1]  for x in self.target[i]] if self.target[i] else [])

            if self.navigating_to_room is None:
                tags_numbersl = []#
                for i, y in enumerate(self.target):
                    tags_numbers = [x[0]  for x in self.target[i]] if self.target[i] else []
                    tags_numbersl.extend( list(filter(lambda x: x is not None, tags_numbers)))
                print(tags_numbersl)
                self.navigating_to_room = self.target_belongs_to_room[tags_numbersl[0]]

            done_flag = False
            if list(filter(lambda x: abs(x)<45, target_distances)):
                done_flag = True

            if done_flag:

                self.target = [None,None]

                self.status_code = 1
                self.just_switched = True

                self.previousRoom = self.currentRoom
                # self.foundRoomTags=[]

                self.currentRoom = self.navigating_to_room
                self.navigating_to_room = None
                if not self.mapping_done:
                    self.dictAllTags_to_enter[self.currentRoom].append([self.tags_used[0][0],
                                                                        self.tags_used[1][0]])
                    self.room_to_room[self.previousRoom].append(self.currentRoom)
                    self.tags_used = []


            else:
            # If only the right camera sees the ball, rotate clockwise
                if not self.target[0] and self.target[1]:
                    self.drive(self.SLOW+1.54, -self.SLOW)
                # Conversely, rotate counterclockwise
                elif self.target[0] and not self.target[1]:
                    self.drive(self.SLOW, self.SLOW+1)
                # Make the MiRo face the ball if it's visible with both cameras
                elif self.target[0] and self.target[1]:
                    # if self.target[0][1]
                    self.drive(0.5,0.5)
                    # Could centre on target moreso here but probably not necessary
                else:
                    self.drive(self.SLOW, -self.SLOW)
                    # self.status_code = 0  # Go back to square 1...
                    print("MiRo has lost the target...")
                # self.just_switched = True

    def select_room(self):
        print(self.room_to_room)
        print(self.dictAllTags_to_enter)
        room_selection = None
        print(self.currentRoom)
        print(self.previousRoom)
        while True:
            done = False
            print('Please Enter a room to nagigate to:')
            print('Valid Rooms include:', self.room_to_room[self.currentRoom])

            room_selection = str(raw_input('Enter here: '))
            if room_selection in self.room_to_room[self.currentRoom]:
                print('Navigating to room:', room_selection)
                done=True
            else:
                print('Invalid Input')

            if done:
                break

        self.status_code = 5
        self.navigating_to_room = room_selection


    def find_room_door(self):

        if self.just_switched:  # Print once
            print("MiRo is looking for the right door")
            self.just_switched = False

        tags_to_look_for = list(filter(lambda x: x in self.dictAllTags[self.currentRoom],
                                       self.dictAllTags_to_enter[self.navigating_to_room]))
        for index in range(2):  # For each camera (0 = left, 1 = right)
            # Skip if there's no new image, in case the network is choking
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            flat_img = self.rectifyImages(image, index)
            # Run the detect ball procedure
            self.nav_tags[index] = self.detect_AprilTags(flat_img, index)

        for i, y in enumerate(self.nav_tags):
            self.nav_tags[i] = list(filter(lambda x:  x[0] < 13, self.nav_tags[i])) if y else y

        if self.nav_tags[0]:
            tags_identified = [x[0] for x in self.nav_tags[0]]
            if tags_identified in tags_to_look_for:
                self.status_code = 2
                self.nav_tags = [None,None]
                self.just_switched = True
            self.drive(self.SLOW, -self.SLOW)
        else:
            self.drive(self.SLOW, -self.SLOW)





    def __init__(self):
        # Initialise a new ROS node to communicate with MiRo
        if not self.NODE_EXISTS:
            rospy.init_node("kick_blue_ball", anonymous=True)
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)
        # Initialise CV Bridge
        self.image_converter = CvBridge()
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + "miro" # os.getenv(str("MIRO_ROBOT_NAME")) - for IRL implementation
        topicpackage = topic_base_name+"/sensors/package"
        # Create two new subscribers to recieve camera images with attached callbacks
        self.sub_caml = rospy.Subscriber(
            topic_base_name + "/sensors/caml/compressed",
            CompressedImage,
            self.callback_caml,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.sub_camr = rospy.Subscriber(
            topic_base_name + "/sensors/camr/compressed",
            CompressedImage,
            self.callback_camr,
            queue_size=1,
            tcp_nodelay=True,
        )
        # Create a new publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )
        # Create a new publisher to move the robot head
        self.pub_kin = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )
        self.sub_package = rospy.Subscriber(topicpackage, miro.msg.sensors_package,
            self.callback_package
        )
        # Create handle to store images
        self.input_camera = [None, None]
        # New frame notification
        self.new_frame = [False, False]
        # Create variable to store a list of ball's x, y, and r values for each camera
        self.doorway = [None, None]

        self.target = [None, None]
        self.nav_tags = [None, None]
        # Set the default frame width (gets updated on recieving an image)
        self.frame_width = 640
        # Action selector to reduce duplicate printing to the terminal
        self.just_switched = True
        # Bookmark
        self.bookmark = 0
        # Sonar sensor
        self.sonar=None
        # Variables relating to tags
        self.foundRoomTags=[]
        self.tags_used = []
        self.dictAllTags={'A':[[0,1]], 'B':[[2,3]]}

        self.room_to_room={'A':[],'B':[]}

        self.dictAllTags_to_enter={'B':[], 'A':[]}
        self.currentRoom=None

        self.navigating_to_room = None

        self.allRoomTags=[]
        self.previousRoom=None
        self.mapping_done = False
        self.status_code=0
        self.target_belongs_to_room = {14:'B', 15:'A'}
        # Instantiate april tag class

        # Move the head to default pose
        self.reset_head_pose()


    def loop(self):
        """
        Main control loop
        """
        print("MiRo plays ball, press CTRL+C to halt...")
        # Main control loop iteration counter
        self.counter = 0
        self.middle_flag = False
        # This switch loops through MiRo behaviours:
        self.tag = AprilTagPerception(10)
        while not rospy.core.is_shutdown():
            # Step 1. Find all AprilTags in a room
            if self.status_code == 1:
                # Every once in a while, look for ball        self.current_room='A'

                if self.counter % self.CAM_FREQ == 0:
                    self.look_for_AprilTags()

            # Step 2. Move through doorway between two AprilTags
            elif self.status_code == 2:
                self.find_Doorway()

            elif self.status_code == 3:
                self.lock_onto_target()
            elif self.status_code == 4:
                self.select_room()
            elif self.status_code == 5:
                self.find_room_door()
            # Fall back
            else:
                self.status_code = 1

            # Yield
            self.counter += 1
            rospy.sleep(self.TICK)


# This condition fires when the script is called directly
if __name__ == "__main__":
    main = MiRoClient()  # Instantiate class
    main.loop()  # Run the main control loop
