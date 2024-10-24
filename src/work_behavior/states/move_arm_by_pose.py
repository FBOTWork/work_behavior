#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import smach
from geometry_msgs.msg import PoseStamped
import moveit_commander
import sys
from moveit_msgs.msg import RobotState, Constraints

class MoveArmState(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["succeeded", "aborted", "preempted"], input_keys=["arm_target_pose"]
        )
        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        
        # Configure the planner
        self.setup_planner()

    def setup_planner(self):
        """Configure the motion planner with optimal parameters"""
        self.group.allow_replanning(True)
        self.group.set_planning_time(10.0)  # Increased planning time
        self.group.set_num_planning_attempts(20)  # Increased attempts
        # self.group.set_goal_position_tolerance(0.01)  # 1cm tolerance
        # self.group.set_goal_orientation_tolerance(0.1)  # ~5.7 degrees
        
        # Set velocity and acceleration scaling
        self.group.set_max_velocity_scaling_factor(0.5)
        self.group.set_max_acceleration_scaling_factor(0.5)

    def execute(self, userdata):
        try:
            rospy.loginfo("Receiving pose from userdata...")
            
            # Validate userdata
            if "arm_target_pose" not in userdata or not userdata.arm_target_pose:
                rospy.logerr("No pose provided in userdata.")
                return "aborted"
            
            target_pose = userdata.arm_target_pose
            
            rospy.loginfo("Moving arm to position: {}".format(target_pose.pose.position))
            current_pose = self.group.get_current_pose().pose
            rospy.loginfo("Current arm position: {}".format(current_pose))
            
            # Set the target pose
            self.group.set_pose_target(target_pose.pose)
            
            # Try planning with different planners if the default fails
            # planners = ['RRTConnect', 'RRTstar', 'PRMstar']
            planners = ['RRTConnect']

            success = False
            
            for planner in planners:
                rospy.loginfo("Attempting to plan with {}".format(planner))
                self.group.set_planner_id(planner)
                
                # In ROS Kinetic, plan() returns a RobotTrajectory object directly
                plan = self.group.plan()
                
                if plan:
                    rospy.loginfo("Planning succeeded with {}".format(planner))
                    # For ROS Kinetic, we should use go() instead of execute()
                    success = self.group.go(wait=True)
                    if success:
                        break
                    else:
                        rospy.logwarn("Execution failed with {}".format(planner))
                else:
                    rospy.logwarn("Planning failed with {}".format(planner))
            
            # Clean up
            self.group.stop()
            self.group.clear_pose_targets()
            
            if success:
                rospy.loginfo("Arm movement completed successfully")
                return "succeeded"
            else:
                rospy.logerr("Failed to move arm with all planners")
                return "aborted"
                
        except Exception as e:
            rospy.logerr("Error during arm movement: {}".format(str(e)))
            return "aborted"