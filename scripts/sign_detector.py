#!/usr/bin/env python

import numpy as np
import math
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from sign_detection.msg import SignLocationPixel
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class SignDetector():
    """
    A class for applying your sign detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_sign_px (SignLocationPixel) : the coordinates of the sign in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs sign parker
        self.LineFollower = rospy.get_param("is_line_follower", False)
        DRIVE_TOPIC = "/vesc/ackermann_cmd_mux/input/navigation" #rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar

        # Subscribe to ZED camera RGB frames
        self.sign_pub = rospy.Publisher("/relative_sign_px", SignLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/sign_debug_img", Image, queue_size=10)
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images
        self.done = False

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_sign_px topic; the homography transformer will
        # convert it to the car frame.

        if self.done:
            return

        # Convert ROS image to OpenCV Image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        bounding_box = cd_color_segmentation(image, is_line_follower=self.LineFollower)
        
        # Publish detected bounding box
        if bounding_box == ((0,0), (0,0)):
            return
        else:
            sign_location = SignLocationPixel()
            sign_location.u = int((bounding_box[0][0] + bounding_box[1][0]) / 2)
            sign_location.v = bounding_box[1][1]
            self.sign_pub.publish(sign_location)

        #debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        #self.debug_pub.publish(debug_msg)

    def drive(self, speed = 0, steering_angle = 0):
        """
        Publishes AckermannDriveStamped msg with speed, steering_angle, and steering_angle_velocity
        """
        # create drive object
        ack_drive = AckermannDrive()
        ack_drive.speed = speed
        ack_drive.steering_angle = steering_angle

        #create AckermannDriveStamped object
        ack_stamp = AckermannDriveStamped()
        ack_stamp.drive = ack_drive
        self.drive_pub.publish(ack_stamp)


if __name__ == '__main__':
    try:
        rospy.init_node('SignDetector', anonymous=True)
        SignDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
