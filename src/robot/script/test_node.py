#!/usr/bin/env python

# -*- coding: utf-8 -*-
# Path: Valrob_pkg\script\test_node.py
# test de la communication serie avec la carte de controle des moteurs du robot

import rospy # type: ignore
from geometry_msgs.msg import Twist, Vector3# type: ignore
from robot.srv import encoders, encodersResponse # type: ignore
from threading import Thread
import time

class sendConsign(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.__position = 0
        self.__angle = 0
    def run(self):
        dt = time.time()
        while rospy.is_shutdown() == False:
            
            position_pub.publish(Twist(Vector3(self.__position,0,0), Vector3(0,0,self.__angle)))
    

def encoders_client(get_encoders):
    return (get_encoders().x, get_encoders().y)

rospy.init_node("test_node")
consPub = rospy.Publisher('robot_consign', Twist, queue_size=10)
test = sendConsign()

rospy.Subscriber("robot_consign", Twist, test.run)

position_pub = rospy.Publisher('pos', Twist, queue_size=10)

rospy.spin()
