#!/usr/bin/env python
# -*- coding: utf-8 -*- 

# Code used to obtain the pose and ID of the AprilTag closest to the robot.
# It will be used in the Basic Manipulation Test I, II and III.

import rospy
import smach
from apriltag_ros.msg import AprilTagDetectionArray

class GetInfoFromNearestAprilTagState(smach.State):
    def __init__(self):
        # Initializes the SMACH state
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['pose'])
        # Initializes the pose variable as None, indicating that no pose was detected
        self.pose = None
        # Subscribes to the /tag_detections topic
        self.subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)

    def callback(self, msg):
        # Checks if there was any detection
        if len(msg.detections) > 0:
            # Gets the first detection and retrieves the pose
            self.pose = msg.detections[0].pose.pose.pose
            # Gets the first detection and retrieves the ID
            self.id = msg.detections[0].id
            print(self.pose)
            print("ID:", self.id)
        else:
            self.pose = None

    def execute(self, userdata):
        rospy.loginfo('Reading the /tag_detections topic...')
        # Waits until a message is received or timeout occurs
        timeout = rospy.Time.now() + rospy.Duration(5.0)  # 5-second timeout
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.pose is not None: # Checks if the pose was obtained
                userdata.pose = self.pose  # Saves the pose in userdata to be used by other states
                rospy.loginfo('Pose successfully obtained.')
                return 'succeeded'
            rospy.sleep(0.1)  # Waits a bit before trying again

        rospy.loginfo('Failed to obtain the AprilTag pose.')
        return 'aborted'