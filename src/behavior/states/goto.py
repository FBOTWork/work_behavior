#!/usr/bin/env python

import rospy 
import smach 

from move_base_msgs.msg import MoveBaseGoal

from behavior.helpers.movebase_client  import create_movebase_client

#Definir estado GoTo

class GoToState(smach.State):
     def __init__(self,frame='map'):
         smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted', 'arrived'], 
                               input_keys=['pose'])
	 self.client = create_movebase_client()
         self.frame = frame


     def execute(self, userdata):
         goal = MoveBaseGoal()
         goal.target_pose.header.frame_id = self.frame
         goal.target_pose.header.stamp = rospy.Time.now()
         goal.target_pose.pose = userdata.pose
         print("GOTO POSE ::: ",userdata.pose)
         self.client.send_goal(goal)
         result = self.client.wait_for_result()

     if self.preempt_requested():
        return 'preempted'

     if result:
        return 'succeeded'
     else:
        return 'aborted'
