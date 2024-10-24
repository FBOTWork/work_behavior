#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
from geometry_msgs.msg import PoseStamped
import moveit_commander
import sys


# Estado para mover o braço com base na pose
class MoveArmState(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["succeeded", "aborted", "preempted"], input_keys=["target_pose"]
        )
        # Inicializar o moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(
            "arm"
        )  # Certifique-se de que o nome do grupo esteja correto

    def execute(self, userdata):
        rospy.loginfo("Recebendo pose do userdata...")

        # Verificar se a pose foi passada corretamente no userdata
        if "target_pose" not in userdata or not userdata.arm_target_pose:
            rospy.logerr("Nenhuma pose foi passada no userdata.")
            return "aborted"

        # Obter a pose do userdata
        target_pose = userdata.arm_target_pose

        rospy.loginfo(
            "Movendo o braço para a posição: {}".format(target_pose.pose.position)
        )

        # Definir a pose como alvo para o MoveIt
        self.group.set_pose_target(target_pose.pose)

        # Planejar e executar o movimento
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        # Verificar se o movimento foi bem-sucedido
        if plan:
            rospy.loginfo("Braço movido com sucesso.")
            return "succeeded"
        else:
            rospy.logerr("Falha ao mover o braço.")
            return "aborted"
