import rospy
import smach

from work_behavior.states import GetInfoFromNearestAprilTagState, MoveArmState


def pickupATTC():
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])

    with sm:
        # Add the MoveArmState to the state machine
        smach.StateMachine.add(
            "MOVE_ARM",
            MoveArmState(),
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",
            },
        )
    # Execute the state machine
    return sm
