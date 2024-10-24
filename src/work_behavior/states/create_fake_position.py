import rospy
import smach
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class CreateFakePositionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['fake_pose'])
        self.pose_publisher = rospy.Publisher('/fake_pose', PoseStamped, queue_size=10)

    def execute(self, userdata):
        fake_pose = PoseStamped()
        # Set the fake pose values
        fake_pose.pose.position.x = 1.0
        fake_pose.pose.position.y = 2.0
        fake_pose.pose.position.z = 0.0
        fake_pose.pose.orientation.x = 0.0
        fake_pose.pose.orientation.y = 0.0
        fake_pose.pose.orientation.z = 0.0
        fake_pose.pose.orientation.w = 1.0

        # Create a marker to visualize the pose in RViz
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "fake_pose"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = fake_pose.pose
        marker.scale.x = 0.2
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        while True:
            marker_publisher.publish(marker)


        # Publish the fake pose
        self.pose_publisher.publish(fake_pose)

        # Store the fake pose in userdata
        userdata.arm_target_pose = fake_pose

        return 'succeeded'