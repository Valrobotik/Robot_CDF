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
    def run(self):
        global message
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            if message != None:
                odomPub.publish(message)

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
        self.__position = [0, 0, 0] #(x, y, theta)
        self.__maxTicks = 65535
        self.__maxSafeTicks = 500
    
        self.__maxTicks_v = 2
        self.__maxTicks_a = 3
    
        self.__getreset = rospy.Subscriber("reset_all", Bool, self.reset)
        
    def reset(self, msg: Bool):
        if msg.data :
            self.__lastTime = rospy.Time.now()
            self.__currentTime = rospy.Time.now()
            self.__position = [0, 0, 0] #(x, y, theta)
            
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
                if(self.__localVelocity[0] != -1  and self.__localVelocity[1] != -1):
                    rospy.logdebug("dt = %f", dt)
                    if dt > 0:
                        #calcul des vitesses et de la position
                        self.FigureSpeed(dt)
                        #envoie des données pour la position estimée
                        velocityPublisher(self.__position[0], self.__position[1], self.__position[2], 
                                        self.__localVelocity[0], self.__localVelocity[1], self.__currentTime)
                        
                self.__lastlocalVelocity = self.__localVelocity #on enregistre les anciennes valeurs de vitesse 
                
    def FigureSpeed(self, dt):
        #calcul de la position à partir des vitesses lineaire et angulaire
        w = self.__localVelocity[1]
        v = self.__localVelocity[0]
        """
        vx = v*math.cos(self.__position[2])
        vy = v*math.sin(self.__position[2])

        dx = vx*dt
        dy = vy*dt
        dth = w*dt

        self.__position[0]+=dx
        self.__position[1]+=dy
        self.__position[2] = self.reduceAngle(self.__position[2] + dth)
        """
        if abs(w) < 0.01:
            self.__position[0] += v*dt*math.cos(self.__position[2])
            self.__position[1] += v*dt*math.sin(self.__position[2])
            self.__position[2] += w*dt
        else:
            self.__position[0] += (v/w)*(math.sin(self.__position[2] + w*dt) - math.sin(self.__position[2]))
            self.__position[1] += (v/w)*(math.cos(self.__position[2]) - math.cos(self.__position[2] + w*dt))
            self.__position[2] += w*dt
        
        self.__position[2] = self.reduceAngle(self.__position[2])
        
        pass
    
    def reduceAngle(self, x): 
        a = math.fmod(x, 2*math.pi)
        return a
       
def velocityPublisher(x, y, th, v, w, t):
    global message
    odomQuat = quaternion_from_euler(0, 0, th)

    message = Odometry()
    message.header.stamp = t
    message.header.frame_id = "odom"
    message.pose.pose = Pose(Point(x, y, 0), Quaternion(*odomQuat))
    message.child_frame_id = "base_link"
    vx = v*math.cos(th)
    vy = v*math.sin(th)
    message.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, w))
    #print(message)
    pass

def encoders_client():
    rospy.wait_for_service('encoders')
    get_encoders = rospy.ServiceProxy('encoders', encoders)
    cmd = 1
    rep = get_encoders(cmd)
    return (rep.left, rep.right)



message : Odometry = None

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