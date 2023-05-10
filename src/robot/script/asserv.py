#!/usr/bin/env python
#asservicement en position du robot

import rospy
import time
from nav_msgs.msg import Odometry # type: ignore
from geometry_msgs.msg import Twist, Vector3
import math
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler # type: ignore

class position():
    def __init__(self) -> None:
        
        rospy.init_node("asserv", log_level=rospy.DEBUG)
        
        #constantes du PID lineaire
        self.__kpv = 1
        self.__kiv = 0.001
        self.__kdv = 0
        
        #constantes du PID angulaire
        self.__kpa = 5
        self.__kia = 0.1
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
        
        self.go_a = 0
        self.go_x = 0
        self.go_y = 0
            
        #rospy.Subscriber("/odometry/filtered", Odometry, self.odom)
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
        (roll, pitch, yaw) = euler_from_quaternion([rep.pose.pose.orientation.x, rep.pose.pose.orientation.y, rep.pose.pose.orientation.z, rep.pose.pose.orientation.w])
        self.a = yaw
        rospy.logdebug("x = %f, y = %f, a = %f", self.x, self.y, self.a)
        
    def rotation(self, angle):
        consigne = Twist()
        consigne.linear.x = 0
        consigne.linear.y = 0
        consigne.angular.z = 3
        self.__integral_a = 0
        self.__previous_error_a  = angle - self.a
        previous_time = time.time()
        while abs(self.a - angle) > self.error_a:
            self.__dt = time.time() - previous_time
            self.pub.publish(consigne)
            consigne.angular.x = self.pid_a(angle - self.a)
            rospy.sleep(0.02)
        self.stop()
    
    def translation(self, x, y):
        consigne = Twist()
        consigne.linear.y = 0
        consigne.angular.z = 3
        previous_time = time.time()
        while abs(self.x - x) > self.error_l or abs(self.y - y) > self.error_l:
            self.__dt = time.time() - previous_time
            consigne.linear.x = self.pid_v((x - self.x)**2 + (y - self.y)**2)
            if consigne.linear.x > 0.8: consigne.linear.x = 0.8
            consigne.angular.x = self.pid_a(math.atan2(y - self.y, x - self.x)-self.a)
            self.pub.publish(consigne)
            rospy.sleep(0.1)
        self.stop()
        
    def pid_v(self, erreur):
        self.__integral_v += erreur*self.__dt
        if self.__integral_v > 0.2: self.__integral_v = 0.2
        self.__derivative_v = (erreur - self.__previous_error_v)/self.__dt
        self.__previous_error_v = erreur
        return(self.__kpv*erreur + self.__kiv*self.__integral_v + self.__kdv*self.__derivative_v)
    
    def pid_a(self, erreur):
        temp_integral = self.__integral_a
        self.__integral_a += erreur*self.__dt*self.__kia
        self.__proportional_a = self.__kpa*erreur
        self.__derivative_a = self.__kda*(erreur - self.__previous_error_a)/self.__dt
        self.__previous_error_a = erreur
        comande = self.__proportional_a + self.__integral_a + self.__derivative_a
        #saturation avec anti windup
        if comande > 1:
            comande = 1
            self.__integral_a = temp_integral
        elif comande < -1:
            comande = -1
            self.__integral_a = temp_integral
        return(comande)
    
    def stop(self):
        consigne = Twist()
        consigne.linear.x = 0
        consigne.linear.y = 0
        consigne.angular.z = 0
        self.pub.publish(consigne)
        rospy.sleep(0.1)
    
    def go_to(self):
        while not rospy.is_shutdown():
            #self.rotation(math.atan2(y, x))
            self.rotation(self.go_a)
            #self.translation(x, y)

    def go(self, rep):
        self.go_x = rep.x
        self.go_y = rep.y
        self.go_a = rep.z

pos = position()
pos.go_to()
rospy.spin()