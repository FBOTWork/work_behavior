#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import smach
from work_behavior.states import GetInfoFromNearestAprilTagState

if __name__ == '__main__':
  rospy.init_node('test_apriltag')  
  sm = smach.StateMachine(outcomes = ['succeeded', 'aborted'])
  with sm:
    smach.StateMachine.add('GET_APRILTAG', GetInfoFromNearestAprilTagState(), 
    transitions = {
      'succeeded': 'succeeded',
      'aborted': 'GET_APRILTAG',
      #'preempted': 'preempted',
    })
  outcome = sm.execute()
