import smach
import rospy
from butia_world_msgs.srv import GetPose
import math

def euclid_dist(p1, p2):
  return math.sqrt(math.pow(p1.x - p2.x, 2)+math.pow(p1.y - p2.y, 2)+math.pow(p1.z - p2.z, 2))

class GetTargetPoseState(smach.State):
  def __init__(self, service='/butia_world/get_pose'):
    smach.State.__init__(self, outcomes=['succeeded', 'error', 'arrived'], 
                               input_keys=['key'],
                               output_keys=['pose', 'size'])
    self.service = service
    self.client = rospy.ServiceProxy(self.service, GetPose)

  def execute(self, userdata):
    # print('target/' + userdata.key + '/pose')
    rospy.wait_for_service(self.service)
    rospy.loginfo(userdata.key)
    try:
      
      response = self.client('target/' + userdata.key + '/pose')
      userdata.pose = response.pose
      userdata.size = response.size
      return 'succeeded'
    except rospy.ServiceException as e:
      rospy.logerr("Service call failed: %s"%e)
      return 'error'
    