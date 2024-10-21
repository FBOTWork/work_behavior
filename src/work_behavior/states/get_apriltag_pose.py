#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import smach
from apriltag_ros.msg import AprilTagDetectionArray

class GetInfoFromNearestAprilTagState(smach.State):
    def __init__(self):
        # Inicializa o estado SMACH com o resultado 'succeeded' ou 'aborted'
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['id'], output_keys=['pose'])
        self.pose = None
        # Assina o tópico /tag_detections
        self.subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)

    def callback(self, msg):
        # Verifica se houve alguma detecção
        if len(msg.detections) > 0:
            # Pega a primeira detecção e obtém a pose
            self.pose = msg.detections[0].pose.pose.pose
            self.id = msg.detections[0].id
            print(self.pose)
            print("ID:", self.id)
        else:
            self.pose = None

    def execute(self, userdata):
        rospy.loginfo('Lendo o tópico /tag_detections...')
        # Espera até receber uma mensagem ou timeout
        timeout = rospy.Time.now() + rospy.Duration(5.0)  # Timeout de 5 segundos
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.pose is not None:
                userdata.pose = self.pose  # Salva a pose em userdata
                rospy.loginfo('Pose obtida com sucesso.')
                return 'succeeded'
            rospy.sleep(0.1)  # Espera um pouco antes de tentar novamente

        rospy.loginfo('Falha ao obter a pose do AprilTag.')
        return 'aborted'