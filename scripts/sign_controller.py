#!/usr/bin/env python

import rospy
import numpy as np
import math

from sign_detection.msg import SignLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String

class SignController():
    """
    A controller for parking in front of a sign.
    Listens for a relative sign location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_sign", SignLocation,
            self.relative_sign_callback)

        DRIVE_TOPIC = rospy.get_param("/vesc/low_level/ackermann_cmd_mux/input/safety") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)

        self.LineFollower = rospy.get_param("is_line_follower", False)

        self.relative_x = 0
        self.relative_y = 0

        self.last_detection = rospy.get_time()
        self.stop_time = rospy.get_time()
        self.is_stopped = False


    def relative_sign_callback(self, msg):
        """
        Park car facing object given x_pos and y_pos
        TODO: Find way to vary approach velocity using safety controller code
        TODO: Find appropriate velocity for backing up
        TODO: Test to make sure sign kept in field of vision. 
            Reduce Lfwb if sign not kept in lookahead distance
        TODO: Workout trig to find minimum distance needed to avoid more than
        one two point turn
        """

        self.relative_x = msg.x_pos
        current_time = rospy.get_time()
        
        if self.is_stopped or (self.relative_x >= 0.75 and self.relative_x <= 1.0 and abs(current_time - self.last_detection) > 10):
            if not self.is_stopped:
                self.is_stopped = True
                self.stop_time = rospy.get_time()
            if abs(current_time - self.stop_time < 1.0):
                self.drive()
            else:
                self.is_stopped = False
                self.last_detection = rospy.get_time()

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
        rospy.init_node('SignController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
