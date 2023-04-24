#!/usr/bin/env python

# -*- coding: utf-8 -*-
# Path: Valrob_pkg\script\go_pos.py
# permet de faire avancer le robot a une position donnee

from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import rospy

class asservicement():
    def __init__(self) -> None:
        #initialisation des constantes de l'asservicement carte moteur
        self.Kp_l = 0.1
        self.Ki_l = 0.1
        self.Kd_l = 0.1
        
        self.kp_r = 0.1
        self.ki_r = 0.1
        self.kd_r = 0.1
        
        #publisheur des constantes de l'asservicement
        param_publisher = rospy.Publisher('server_req', String, queue_size=10) #initialisation du publisher
        
        #preparation des requetes au moteur
        moteur_droit = "{type: 'motor_request', request: 'set_pid_left', p: "+str(self.kp_l)+", i: "+str(self.ki_l)+", d: "+str(self.kd_l)+"}"
        moteur_gauhe = "{type: 'motor_request', request: 'set_pid_right', p: "+str(self.Kp_r)+", i: "+str(self.Ki_r)+", d: "+str(self.Kd_r)+"}"
        
        #envoi des requetes
        param_publisher.publish(moteur_droit)
        rospy.sleep(0.1)
        param_publisher.publish(moteur_gauhe)
        rospy.sleep(0.1)
        
        #initialisation des constantes de l'asservicement position
        self.K = 0.1
        self.Ti = 0.1
        self.Td = 0.1
        
        #initialisation des variables de l'asservicement
        self.x = 0
        self.y = 0
        self.theta = 0
        self.ex = 0
        self.ey = 0
        self.etheta = 0
        
        self.position_sub = rospy.Subscriber('odometry', Vector3, self.set_pos)
        self.go_to = rospy.Subscriber('go_to', Vector3, self.set_pos)
    
    def set_pos(self, pos):
        self.x = pos.x
        self.y = pos.y
        self.theta = pos.z
        return