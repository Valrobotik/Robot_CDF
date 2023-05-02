#!/usr/bin/env python
#asservicement en position du robot

import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import math
from std_msgs.msg import Bool

class position():
    def __init__(self) -> None:
        
        rospy.init_node("asserv")
        
        #constantes du PID lineaire
        self.__kpv = 1
        self.__kiv = 0.001
        self.__kdv = 0
        
        #constantes du PID angulaire
        self.__kpa = -2
        self.__kia = -0.001
        self.__kda = 0
        
        #erreurs lineaire et angulaire tolerees pour la fin du PID
        self.error_l = 0.1
        self.error_a = 0.01
        
        #variables du PID lineaire
        self.__integral_v = 0
        self.__integral_a = 0
        
        self.__previous_error_v = 0
        self.__previous_error_a = 0
        
        self.__dt = 0
        
        self.__action = False
        
        #position du robot
        self.x = 0
        self.y = 0
        self.a = 0

        rospy.Subscriber("/Odom", Odometry, self.odom)
        self.pub = rospy.Publisher("robot_consign", Twist, queue_size=10)
        rospy.Subscriber("go", Vector3, self.go)
        rospy.Subscriber("reset_all", Bool, self.reset)
    
    def reset(self, rep:Bool):
        if rep.data:
            self.__integral_v = 0
            self.__integral_a = 0  
            self.__previous_error_v = 0
            self.__previous_error_a = 0
            self.__dt = 0
            self.__action = False
            
            #position du robot
            self.x = 0
            self.y = 0
            self.a = 0
        
    
    def odom(self, rep):
        self.x = rep.pose.pose.position.x
        self.y = rep.pose.pose.position.y
        self.a = rep.pose.pose.orientation.z
        
    def rotation(self, angle):
        consigne = Twist()
        consigne.linear.x = 0
        consigne.linear.y = 0
        consigne.angular.z = 3
        previous_time = time.time()
        while abs(self.a - angle) > self.error_a:
            self.__dt = time.time() - previous_time
            consigne.angular.x = self.pid_a(self.a - angle)
            self.pub.publish(consigne)
            time.sleep(0.1)
    
    def translation(self, x, y):
        consigne = Twist()
        consigne.linear.y = 0
        consigne.angular.z = 3
        previous_time = time.time()
        while abs(self.x - x) > self.error_l or abs(self.y - y) > self.error_l:
            self.__dt = time.time() - previous_time
            consigne.linear.x = self.pid_v((x - self.x)**2 + (y - self.y)**2)
            consigne.angular.x = self.pid_a(math.atan2(y - self.y, x - self.x)-self.a)
            self.pub.publish(consigne)
            time.sleep(0.1)
        
    def pid_v(self, erreur):
        self.__integral_v += erreur*self.__dt
        if self.__integral_v > 0.5: self.__integral_v = 0.5
        self.__derivative_v = (erreur - self.__previous_error_v)/self.__dt
        self.__previous_error_v = erreur
        return(self.__kpv*erreur + self.__kiv*self.__integral_v + self.__kdv*self.__derivative_v)
    
    def pid_a(self, erreur):
        self.__integral_a += erreur*self.__dt
        if self.__integral_a > 0.5: self.__integral_a = 0.5
        self.__derivative_a = (erreur - self.__previous_error_a)/self.__dt
        self.__previous_error_a = erreur
        return(self.__kpa*erreur + self.__kia*self.__integral_a + self.__kda*self.__derivative_a)
    
    def stop(self):
        consigne = Twist()
        consigne.linear.x = 0
        consigne.linear.y = 0
        consigne.angular.z = 0
        self.pub.publish(consigne)
        time.sleep(0.1)
    
    def go_to(self, x, y):
        self.rotation(math.atan2(y, x))
        self.stop()
        self.translation(x, y)

    def go(self, rep):
        if rep.x != None and rep.y != None:
            self.go_to(rep.x, rep.y)
        if rep.z != None:
            self.rotation(rep.z)
        self.stop()

pos = position()
rospy.spin()