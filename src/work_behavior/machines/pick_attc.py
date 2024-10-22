import rospy
import smach

from work_behavior.states import GetInfoFromNearestAprilTagState, MoveArmState

def pickupATTC():
    rospy.init_node('test_move_arm_state')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Define the target position for the arm
    target_position = {
        'x': 0.5,
        'y': 0.0,
        'z': 0.5,
        'roll': 0.0,
        'pitch': 0.0,
        'yaw': 0.0
    }

    with sm:
        # Add the MoveArmState to the state machine
        smach.StateMachine.add('MOVE_ARM', MoveArmState(target_position),
                               transitions={'succeeded': 'succeeded',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})

    # Execute the state machine
    outcome = sm.execute()

if __name__ == '__main__':
    pickupATTC()