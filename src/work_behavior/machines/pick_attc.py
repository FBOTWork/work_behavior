import rospy
import smach

from work_behavior.states import GetInfoFromNearestAprilTagState, MoveArmState, CreateFakePositionState


def pickupATTC():

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])

    with sm:

        # Add the FakePositionState to the state machine
        smach.StateMachine.add(
            "CREATE_FAKE_POSITION",
            CreateFakePositionState(),
            transitions={"succeeded": "MOVE_ARM"},
        )

        # Add the MoveArmState to the state machine
        smach.StateMachine.add(
            "MOVE_ARM",
            MoveArmState(),
            transitions={
                "succeeded": "succeeded",
                "aborted": "aborted",
                "preempted": "preempted",
            },
        )
    # Execute the state machine
    return sm
