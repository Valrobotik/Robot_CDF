#!/usr/bin/env python

# -*- coding: utf-8 -*-
# Path: Valrob_pkg\script\test_node.py
# test de la communication serie avec la carte de controle des moteurs du robot

import rospy
from geometry_msgs.msg import Twist, Vector3
from threading import Thread

class sendConsign(Thread):
    def __init__(self):
        Thread.__init__(self)
    def run(self):
        while True:
            consPub.publish(Twist(Vector3(0.5,0,0), Vector3(0,0,3)))
            rospy.sleep(1)

rospy.init_node("test_node")
consPub = rospy.Publisher('robot_consign', Twist, queue_size=10)

test = sendConsign()
test.start()

rospy.spin()
