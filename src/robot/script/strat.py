#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

class Match():
    def __init__(self) -> None:
        self.__run = True # match is not running
        rospy.init_node("match") #initialisation du noeud match
        rospy.Subscriber("match", Bool, self.matchCallback) #initialisation du subscriber qui recupere le lancement du match
        self.__reset = rospy.Publisher("reset_all", Bool, queue_size=10) #initialisation du publisher qui reset les noeuds du robot
        self.__control = rospy.Publisher("go", Vector3, queue_size=10) #initialisation du publisher qui permet de controler le robot
        
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
                
    def run(self):
        rospy.loginfo("match start")
        rospy.sleep(5)
        while self.__run and not rospy.is_shutdown(): #boucle de match
            goto = Vector3(1,0,0) #(x,y,theta)
            self.__control.publish(goto) #envoi de la commande de controle du robot
            rospy.sleep(1) #attente de la fin du match
            
match = Match() #initialisation du match
match.run() #lancement 