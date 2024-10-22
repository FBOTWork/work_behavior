#!/usr/bin/env python
# coding: utf-8

import rospy
import smach
from work_behavior.machines import pickupATTC

if __name__ == "__main__":
    rospy.init_node("test_move_arm_state")
    sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])
    with sm:
        smach.StateMachine.add(
            "PICKUP_ATTC",
            pickupATTC(),
            transitions={
                "succeeded": "succeeded",
                "aborted": "aborted",
                "preempted": "preempted",
            },
        )

    outcome = sm.execute()
