import moveit_commander
import moveit_commander.robot_trajectory
from moveit_commander import PlanningSceneInterface
from moveit_commander.robot_trajectory import RobotTrajectory
from moveit_commander.robot_trajectory import RobotState
import rospy
from geometry_msgs.msg import Pose

def move_arm_to_pose(pose):
    # Inicialize o MoveIt!
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")  # Troque "arm" pelo nome correto do seu grupo de movimento

    # Define a pose de destino para o braço
    target_pose = Pose()
    target_pose.position = pose.position
    target_pose.orientation = pose.orientation

    group.set_pose_target(target_pose)
    
    # Planejamento e execução do movimento
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    return plan