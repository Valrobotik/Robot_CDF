#!/usr/bin/env python

# -*- coding: utf-8 -*-
# Path: Valrob_pkg\script\imu.py
# recupere les donees de l'IMU et les publie sur le topic imu_data

import rospy # type: ignore
from geometry_msgs.msg import Twist, Vector3, Quaternion # type: ignore
from BMI160_i2c import Driver # type: ignore
from sensor_msgs.msg import Imu # type: ignore
import math


class IMU():
    def __init__(self) -> None:
        self.__adresse = 0x69 #adresse de l'IMU
        self.__freq = 100    #frequence de publication des donnees
        try:
            self.__sensor = Driver(self.__adresse) #initialisation de l'IMU
            self.__sensor.setAccelOffsetEnabled(True)
            self.__sensor.setGyroOffsetEnabled(True)

            self.__sensor.autoCalibrateXAccelOffset(0)
            self.__sensor.autoCalibrateYAccelOffset(0)
            self.__sensor.autoCalibrateZAccelOffset(1)
            self.__sensor.autoCalibrateGyroOffset()
        except:
            pass

    def run(self):
        while rospy.is_shutdown() == False: #boucle de publication des donnees tant que le noeud n'est pas arrete
            try:
                data = self.__sensor.getMotion6() #recuperation des donnees
            except:
                data = [0,0,0,0,0,0]
            gyro = {
                'x' : data[0] / 262.4 * math.pi / 180,
                'y' : data[1] / 262.4 * math.pi / 180,
                'z' : data[2] / 262.4 * math.pi / 180,
            }
            accel = {
                'x' : data[3] / 16383 * 9.81,
                'y' : data[4] / 16383 * 9.81,
                'z' : data[5] / 16383 * 9.81
            }
            
            message = Imu()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "odom"
            #message.child_frame_id = "base_link"
            message.linear_acceleration = Vector3(accel['x'], accel['y'], accel['z'])
            message.angular_velocity = Vector3(gyro['x'], gyro['y'], gyro['z'])
            orientation = Quaternion()
            #calcul de l'orientation

            publisher.publish(message) #publication des donnees
            rospy.sleep(1/self.__freq) #attente de la frequence de publication

imu = IMU() #initialisation de l'IMU           
rospy.init_node("imu") #initialisation du noeud
publisher = rospy.Publisher('imu_data', Imu, queue_size=10)   #initialisation du publisher
imu.run()  #lancement de la boucle de publication