#!/usr/bin/env python

import rospy

from move_base.msgs.msg import MoveBaseAction

import actionlib

movebase_client = None


def create_movebase_client(timeout=1200):
    global movebase_client
    if movebase_client is None:
        movebase_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        movebase_client.wait_for_server(timeout=rospy.Duration(timeout))
    return movebase_client
