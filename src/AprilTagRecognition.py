#!/usr/bin/env python
"""
Leader-follower flocking for MiRo
Under construction.


The code was tested for Python 2 and 3
For Python 3 you might need to change the shebang line to
#!/usr/bin/env python3
"""

# Imports
##########################
import os
import sys
sys.path.append('common')
from math import radians  # This is used to reset the head pose
import math
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library
import apriltag
from itertools import chain
import collections

import rospy  # ROS Python interface
from sensor_msgs.msg import CompressedImage  # ROS CompressedImage message
from sensor_msgs.msg import JointState  # ROS joints state message
from cv_bridge import CvBridge, CvBridgeError  # ROS -> OpenCV converter
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control) message
import random
import miro2 as miro  # Import MiRo Developer Kit library
import camera_configs
from apriltag_perception import AprilTagPerception

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

    def rectifyImages(self, image, index): # either left or right image
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
        tagIds = []

        if tags:
            for i in range (len(tags)):
                tagIds.append(tags[i].id)
                print(tags[i].family, tags[i].id)
	return tagIds


    def look_for_AprilTags(self):
        """
        [1 of 3] Wander MiRo if it doesn't see an AprilTag in its current
        position, until it sees one.
        """
        if self.just_switched:  # Print once
            print("MiRo is looking for the AprilTags in the room...")
            self.just_switched = False

        if self.new_frame[0]:
            image = self.input_camera[0]
            flat_img = self.rectifyImages(image, 0)
            # Run the detect AprilTag procedure
            detected = self.detect_AprilTags(flat_img, 0)

            if len(detected) > 0 and len(self.foundRoomTags) == 0:
                for key in self.dictAllTags:
                    vals = self.dictAllTags.get(key)
                    flat = list(chain.from_iterable(vals))
                    print(flat)
                    if detected[0] in flat:
                        self.currentRoom = key
                        print(key)
                        self.allRoomTags = flat
                        break

            for tag in detected:
                # in simulation it recognises tags in other rooms through doorways
                if tag not in self.foundRoomTags and tag in self.allRoomTags:
                    self.foundRoomTags.append(tag)
        # Have all AprilTags been detected?
        if sorted(self.foundRoomTags) == sorted(self.allRoomTags):
            if len(self.allRoomTags) > 0:
                print("Found all tags in room")
                print(self.foundRoomTags, self.allRoomTags)
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
            # Run the detect ball procedure
            self.doorway[index] = self.detect_AprilTags(image, index)
        # If only the right camera sees the ball, rotate clockwise
        if len(self.doorway[0]) == 2 and len(self.doorway[1]) == 2:
            self.drive(self.FAST, self.FAST)
        # Conversely, rotate counterclockwise
        elif self.ball[0] and not self.ball[1]:
            self.drive(-self.SLOW, self.SLOW)
        # Make the MiRo face the ball if it's visible with both cameras
        elif not self.doorway[0] and self.doorway[1]:
            self.drive(self.SLOW, -self.SLOW)
        elif len(self.doorway[0] > 0) and len(self.dooorway[1] > 0):
            self.drive(self.SLOW, self.SLOW)
        # Otherwise, the ball is lost :-(
        else:
            self.drive(self.SlOW, -self.SLOW)



    def __init__(self):
        # Initialise a new ROS node to communicate with MiRo
        if not self.NODE_EXISTS:
            rospy.init_node("kick_blue_ball", anonymous=True)
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)
        # Initialise CV Bridge
        self.image_converter = CvBridge()
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + "miro"# os.getenv(str("MIRO_ROBOT_NAME"))
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
        # Set the default frame width (gets updated on reciecing an image)
        self.frame_width = 640
        # Action selector to reduce duplicate printing to the terminal
        self.just_switched = True
        # Bookmark
        self.bookmark = 0
        # Sonar sensor
        self.sonar=None
        self.foundRoomTags=[]
        self.dictAllTags={'A':[[0,1],[2,3]], 'B':[[4,5],[6,7]], 'C':[[8,9],[10,11]]}
        self.currentRoom=None
        self.allRoomTags=[]
        # Move the head to default pose
        self.reset_head_pose()
	self.tag = AprilTagPerception(10)


    def loop(self):
        """
        Main control loop
        """
        print("MiRo plays ball, press CTRL+C to halt...")
        # Main control loop iteration counter
        self.counter = 0
        # This switch loops through MiRo behaviours:
        # Find ball, lock on to the ball and kick ball
        self.status_code = 0
        while not rospy.core.is_shutdown():

            # Step 1. Find ball
            if self.status_code == 1:
                # Every once in a while, look for ball
                if self.counter % self.CAM_FREQ == 0:
                    self.look_for_AprilTags()

            # Step 2. Approach
            elif self.status_code == 2:
                self.find_Doorway()

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
