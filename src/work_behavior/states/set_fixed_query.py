import smach
import rospy

class SetFixedQueryState(smach.State):
  def __init__(self, query):
    smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], 
                               output_keys=['query'])
    self.query = query

  def execute(self, userdata):
    if self.preempt_requested():
      return 'preempted'
    userdata.query = self.query
    return 'succeeded'
  
  def request_preempt(self):
      super().request_preempt()
