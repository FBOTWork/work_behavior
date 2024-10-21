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
    smach.StateMachine.add('GOTO_WAYPOINT2', getGoToFixedMachine('container_table'), 
    transitions = {
      'succeeded': 'succeeded',
      'aborted': 'GOTO_WAYPOINT2',
      'preempted': 'preempted',
      'arrived': 'arrived'
    })
  outcome = sm.execute()
    smach.StateMachine.add('GOTO_WAYPOINT3', getGoToFixedMachine('precision_table'), 
    transitions = {
      'succeeded': 'succeeded',
      'aborted': 'GOTO_WAYPOINT3',
      'preempted': 'preempted',
      'arrived': 'arrived'
    })
    smach.StateMachine.add('GOTO_WAYPOINT4', getGoToFixedMachine('finish_area'), 
    transitions = {
      'succeeded': 'succeeded',
      'aborted': 'GOTO_WAYPOINT4',
      'preempted': 'preempted',
      'arrived': 'arrived'
    })

