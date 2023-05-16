#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy # type: ignore
from std_msgs.msg import Bool# type: ignore
from geometry_msgs.msg import Vector3# type: ignore
from nav_msgs.msg import Odometry # type: ignore
from tf.transformations import euler_from_quaternion # type: ignore
import math

class pathPlaner():
    def __init__(self) -> None:
        self.x = 0
        self.y = 0
        self.theta = 0

        rospy.init_node("pathPlaner")

        rospy.Subscriber("Odom", Odometry, self.odomCallback)
        rospy.Subscriber("ordre", Vector3, self.ordreCallback)
        rospy.Subscriber("ultra_son", Vector3, self.usCallback)
        self.__control = rospy.Publisher("go", Vector3, queue_size=10)
        self.__break = rospy.Publisher("break", Bool, queue_size=10)

    def odomCallback(self, msg:Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.a = self.mod_2pi(yaw)
    
    def mod_2pi(self, a):
        """retourne l'angle entre -pi et pi"""
        while a > math.pi:
            a -= 2*math.pi
        while a < -math.pi:
            a += 2*math.pi
        return a

    def ordreCallback(self, msg:Vector3):
        """pour l'instant on transmet l'ordre directement"""
        self.__control.publish(msg)

    def usCallback(self, msg):
        """traitement des données des capteurs ultrasons"""
        pass #TODO
        