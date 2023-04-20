#!/usr/bin/env python

# -*- coding: utf-8 -*-
# Path: Valrob_pkg\script\imu.py
# recupere les donees de l'IMU et les publie sur le topic imu_data

import rospy
from geometry_msgs.msg import Twist, Vector3
from BMI160_i2c import Driver

class IMU():
    def __init__(self) -> None:
        self.__adresse = 0x69 #adresse de l'IMU
        self.__freq = 100    #frequence de publication des donnees
        self.__sensor = Driver(self.__adresse) #initialisation de l'IMU
    
    def run(self):
        while rospy.is_shutdown() == False: #boucle de publication des donnees tant que le noeud n'est pas arrete
            data = self.__sensor.getMotion6() #recuperation des donnees
            publisher.publish(Twist(Vector3(data[3],data[4],data[5]), Vector3(data[0],data[1],data[2]))) #publication des donnees
            rospy.sleep(1/self.__freq) #attente de la frequence de publication

imu = IMU() #initialisation de l'IMU           
rospy.init_node("imu") #initialisation du noeud
publisher = rospy.Publisher('imu_data', Twist, queue_size=10)   #initialisation du publisher
imu.run()  #lancement de la boucle de publication