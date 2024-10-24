import rospy
import smach
from geometry_msgs.msg import Pose

class CreateFakePositionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['fake_pose'])
        self.pose_publisher = rospy.Publisher('/fake_pose', Pose, queue_size=10)

    def execute(self, userdata):
        fake_pose = Pose()
        # Set the fake pose values
        fake_pose.position.x = 1.0
        fake_pose.position.y = 2.0
        fake_pose.position.z = 0.0
        fake_pose.orientation.x = 0.0
        fake_pose.orientation.y = 0.0
        fake_pose.orientation.z = 0.0
        fake_pose.orientation.w = 1.0

        # Publish the fake pose
        self.pose_publisher.publish(fake_pose)

        # Store the fake pose in userdata
        userdata.arm_target_pose = fake_pose

        return 'succeeded'