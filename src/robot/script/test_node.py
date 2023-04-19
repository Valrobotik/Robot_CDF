#!/usr/bin/env python

# -*- coding: utf-8 -*-
# Path: Valrob_pkg\script\test_node.py
# test de la communication serie avec la carte de controle des moteurs du robot

import rospy
from geometry_msgs.msg import Twist, Vector3
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
            x, a = encoders_client()
            self.__position += 0.001*x*(time.time()-dt)
            self.__angle += 0.001*a*(time.time()-dt)
            dt = time.time()
            position_pub.publish(Twist(Vector3(self.__position,0,0), Vector3(0,0,self.__angle)))
            if self.__position < 1:
                consPub.publish(Twist(Vector3(0.5,0,0), Vector3(0,0,3)))
            else :
                consPub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
            rospy.sleep(1)


def encoders_client():
    rospy.wait_for_service('encoders')
    get_encoders = rospy.ServiceProxy('encoders', encoders)
    cmd = 1
    rep = get_encoders(cmd)
    return (rep.left, rep.right)

rospy.init_node("test_node")
consPub = rospy.Publisher('robot_consign', Twist, queue_size=10)

position_pub = rospy.Publisher('pos', Twist, queue_size=10)

test = sendConsign()
test.start()

rospy.spin()
