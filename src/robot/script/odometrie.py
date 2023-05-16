#!/usr/bin/env python
from collections.abc import Callable, Iterable, Mapping
from typing import Any
import rospy
import time
import json
from threading import Thread
from robot.srv import encoders, encodersResponse # type: ignore
from nav_msgs.msg import Odometry # type: ignore
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf # type: ignore
import sys
import math
from tf.transformations import quaternion_from_euler # type: ignore
from std_msgs.msg import Bool
from threading import Thread
#lecture fichier de config

class publisher_Tread(Thread):
    def __init__(self):
        super().__init__()
        self.rate = rospy.Rate(30) #100Hz
    def run(self):
        global message
        while not rospy.is_shutdown():
            if message != None:
                odomPub.publish(message)
            self.rate.sleep()

class position():
    x :float = 0
    y : float = 0
    theta : float = 0

class odometrieProcess(Thread):
    def __init__(self):
        super().__init__()
        self.__d = 274.4
        self.__r = 32
        self.__tpr = 1000
        self.__lastTime = 0
        self.__currentTime = 0
        self.__localVelocity = [-1, -1] #(vx, w)
        self.__lastlocalVelocity = [0, 0]
        self.__position = position()
        self.__maxTicks = 65535
        self.__maxSafeTicks = 500
    
        self.__maxTicks_v = 2
        self.__maxTicks_a = 3
    
        self.__getreset = rospy.Subscriber("reset_all", Bool, self.reset)
        
    def reset(self, msg: Bool):
        if msg.data :
            self.__lastTime = rospy.Time.now()
            self.__currentTime = rospy.Time.now()
            self.__position.x = 0
            self.__position.y = 0
            self.__position.theta = 0
            
    def run(self):
        dt = 0
        self.__lastTime = rospy.Time.now()
        while(True):
            data = encoders_client() #on récupère les données des encodeurs
            if data[0] < self.__maxTicks_v and data[1] < self.__maxTicks_a: #on verifie que les valeurs sont correctes
                self.__localVelocity = [data[0], data[1]] #on enregistre les nouvelles valeurs de vitesse
                
                #temps actuel
                self.__currentTime = rospy.Time.now()
                
                #calcul de dt
                dt =  (self.__currentTime-self.__lastTime).to_sec()
                
                #sauvegarde du temps dans une autre variable
                self.__lastTime = self.__currentTime
                
                if(self.__localVelocity[0] == -1 and self.__localVelocity[1] == -1): self.__localVelocity = self.__lastlocalVelocity
                
                if dt > 0:
                    #calcul de la position à partir des vitesses lineaire et angulaire
                    w = (self.__localVelocity[1]+self.__lastlocalVelocity[1])/2
                    v = (self.__localVelocity[0]+self.__lastlocalVelocity[0])/2

                    if abs(w) < 0.01:
                        dx : float = v*dt*math.cos(self.__position.theta)
                        dy : float = v*dt*math.sin(self.__position.theta)
                    else:
                        dx : float = (v/w)*(math.sin(self.__position.theta + w*dt) - math.sin(self.__position.theta))
                        dy : float = (v/w)*(math.cos(self.__position.theta) - math.cos(self.__position.theta + w*dt))
                    
                    self.__position.x += dx
                    self.__position.y += dy
                    self.__position.theta = self.reduceAngle(self.__position.theta + w*dt)

                    #envoie des données pour la position estimée
                    prepare_velocity(self.__position, self.__localVelocity[0], self.__localVelocity[1], self.__currentTime)
                    
                self.__lastlocalVelocity = self.__localVelocity #on enregistre les anciennes valeurs de vitesse 
                
    def reduceAngle(self, x): 
        a = math.fmod(x, 2*math.pi)
        return a
       
def prepare_velocity(pos : position, v:float, w:float, t:rospy.Time):
    """Prepare the velocity message to be published"""
    global message
    odomQuat = quaternion_from_euler(0, 0, pos.theta)
    message.header.stamp = t
    message.pose.pose = Pose(Point(pos.x, pos.y, 0), Quaternion(*odomQuat))
    vx = v*math.cos(pos.theta)
    vy = v*math.sin(pos.theta)
    message.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, w))
    #print(message)
    pass

def encoders_client():
    rospy.wait_for_service('encoders')
    get_encoders = rospy.ServiceProxy('encoders', encoders)
    cmd = 1
    rep = get_encoders(cmd)
    return (rep.left, rep.right)



message = Odometry()
message.header.stamp = 0
message.header.frame_id = "odom"
message.child_frame_id = "base_link"

#initialisation du noeud
rospy.init_node("odometrie", log_level=rospy.INFO)

#publication de l'odometrie
odomPub = rospy.Publisher("Odom", Odometry, queue_size=10)

#initialtisation du tf
odomBroadcaster = tf.TransformBroadcaster()

#filtre de odometrie
odometrie = odometrieProcess()
odometrie.start()

#thread de publication de l'odometrie
pub = publisher_Tread()
pub.start()



#lancement du filtre de odometrie
rospy.logdebug("odometrie process starting ...")

rospy.spin() #boucle infinie