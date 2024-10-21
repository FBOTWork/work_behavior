import smach

import rospy

from geometry_msgs.msg import PoseStamped

from work_behavior.states import SetFixedQueryState, GetTargetPoseState, GoToState
from work_behavior.machines.goto import getGoToMachine

def getGoToFixedMachine(target):
  sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'arrived'])
  with sm:
    smach.StateMachine.add(
        'SET_KEY',
        SetFixedQueryState(target),
        transitions={
        'succeeded': 'GOTO',
        'preempted': 'preempted',
        'aborted': 'aborted'
        },
        remapping={
          'query':'key'
        }
    )
    smach.StateMachine.add(
      'GOTO',
      getGoToMachine(),
      transitions={
        'succeeded': 'succeeded',
        'aborted': 'aborted',
        'preempted': 'preempted',
        'arrived': 'arrived'
      },
      remapping={
        'key': 'key'
      }
    )
  return sm