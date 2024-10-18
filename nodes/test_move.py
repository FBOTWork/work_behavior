#!/usr/bin/env python
# coding: utf-8

import rospy
import smach

from work_behavior.machines import getGoToFixedMachine

if __name__ == '__main__':
  rospy.init_node('navigate_follow')
  sm = smach.StateMachine(outcomes = ['succeeded', 'aborted', 'preempted', 'arrived'])
  with sm:
    smach.StateMachine.add('GOTO_WAYPOINT1', getGoToFixedMachine('service_table'), 
    transitions = {
      'succeeded': 'succeeded',
      'aborted': 'GOTO_WAYPOINT1',
      'preempted': 'preempted',
      'arrived': 'arrived'
    })
  outcome = sm.execute()
