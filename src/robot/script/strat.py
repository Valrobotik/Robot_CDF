#!/usr/bin/env python

import rospy # type: ignore
from std_msgs.msg import Bool# type: ignore
from geometry_msgs.msg import Vector3# type: ignore
from nav_msgs.msg import Odometry # type: ignore
from tf.transformations import euler_from_quaternion # type: ignore
import math

def wait(time):
    rospy.sleep(time)

class Match():
    def __init__(self) -> None:
        self.__run = True # match is not running
        rospy.init_node("match") #initialisation du noeud match
        #initialisation des variables de position
        self.x = 0 
        self.y = 0
        self.theta = 0
        rospy.Subscriber("Odom", Odometry, self.odomCallback)
        rospy.Subscriber("match", Bool, self.matchCallback) #initialisation du subscriber qui recupere le lancement du match
        self.__reset = rospy.Publisher("reset_all", Bool, queue_size=10) #initialisation du publisher qui reset les noeuds du robot
        self.__control = rospy.Publisher("ordre", Vector3, queue_size=10) #initialisation du publisher qui permet de controler le robot
        self.__break = rospy.Publisher("break", Bool, queue_size=10) #initialisation du publisher qui permet de stopper le robot

    def odomCallback(self, msg:Odometry):
        self.x = msg.pose.pose.position.x #recuperation de la position x
        self.y = msg.pose.pose.position.y #recuperation de la position y
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.a = self.mod_2pi(yaw) #angle entre -pi et pi
    
    def mod_2pi(self, a):
        """retourne l'angle entre -pi et pi"""
        while a > math.pi:
            a -= 2*math.pi
        while a < -math.pi:
            a += 2*math.pi
        return a

    def matchCallback(self, msg:Bool):
        rospy.loginfo("match launch : %d",  msg.data) #affichage de l'etat du match
        self.__run = msg.data #recuperation de l'etat du match
    
    def ready(self):
        while not rospy.is_shutdown(): 
            rospy.loginfo("ready for match")
            #self.__reset.publish(True) #reset des noeuds du robot 
            rospy.sleep(0.1)
            if self.__run:
                self.run()
    
    def go(self, x, y, theta = None):
        goto = Vector3(x, y, theta) #(x,y,theta)
        self.__control.publish(goto) #envoi de la commande de controle du robot
    
    def get_x(self):
        return self.x
    def get_y(self):
        return self.y
    def get_theta(self):
        return self.theta
    
    def on_play(self):
        return self.__run

    def stop(self):
        self.__break.publish(True) #envoi de la commande d'arret du robot

    def run(self):
        rospy.loginfo("match start")
        rospy.sleep(10)
        while self.__run and not rospy.is_shutdown(): #boucle de match
            match()

robot = Match() #initialisation du match

def match():
    """code du match à ecrire ici"""
    robot.go(1, 1, 0) #exemple de commande de controle du robot
    rospy.sleep(9)
    robot.go(1, 1, 0) #exemple de commande de controle du robot
    rospy.sleep(9)























m = Match() #initialisation du match
m.run() #lancement 

