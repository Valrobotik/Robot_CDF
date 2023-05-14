#!/usr/bin/env python
#asservicement en position du robot

import rospy# type: ignore
import time
from nav_msgs.msg import Odometry # type: ignore
from geometry_msgs.msg import Twist, Vector3# type: ignore
import math
from std_msgs.msg import Bool# type: ignore
from tf.transformations import euler_from_quaternion, quaternion_from_euler # type: ignore

class position():
    def __init__(self) -> None:
        
        rospy.init_node("asserv", log_level=rospy.DEBUG)
        
        #constantes du PID lineaire
        self.__kpv = 0.6
        self.__kiv = 0.00001
        self.__kdv = 0
        
        #constantes du PID angulaire
        self.__kpa = 3
        self.__kia = 0.1
        self.__kda = 0
        
        #erreurs lineaire et angulaire tolerees pour la fin du PID
        self.error_l = 0.01
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
            previous_time = time.time()
            consigne.angular.x = self.pid_a(angle - self.a)
            self.pub.publish(consigne)
            rospy.sleep(0.02)
        self.stop()
    
    def translation(self, x, y):
        consigne = Twist()
        consigne.linear.y = 0
        consigne.angular.z = 3
        previous_time = time.time()
        while abs(self.x - x) > self.error_l or abs(self.y - y) > self.error_l:
            self.__dt = time.time() - previous_time
            previous_time = time.time()
            consigne.linear.x = self.pid_v(math.sqrt((x - self.x)**2 + (y - self.y)**2))
            err_x = x - self.x
            err_y = y - self.y
            abs_x = abs(err_x)
            abs_y = abs(err_y)
            angle = 0
            #calcul de l'angle a atteindre en fonction de la position du point
            if err_x > 0.001 and err_y > 0:
                angle = math.atan2(abs_y,abs_x)
            elif err_x > 0.001 and err_y < 0:
                angle = -math.atan2(abs_y,abs_x)
            elif err_x < -0.001 and err_y > 0:
                angle = math.pi - math.atan2(abs_y,abs_x)
            elif err_x < -0.001 and err_y < 0:
                angle = math.pi + math.atan2(abs_y,abs_x)
            elif err_x > 0:
                angle = math.pi/2
            elif err_x < 0:
                angle = -math.pi/2
            angle = self.mod_2pi(angle)
            consigne.angular.x = self.pid_a(self.mod_2pi(angle - self.a))
            self.pub.publish(consigne)
            rospy.sleep(0.02)
        self.stop()
        
    def mod_2pi(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return(angle)
        

    def pid_v(self, erreur):
        # the integral term of the PID
        temp_integral = self.__integral_v
        self.__integral_v += erreur*self.__dt*self.__kiv
        # the proportional term of the PID
        self.__proportional_v = self.__kpv*erreur
        # the derivative term of the PID
        self.__derivative_v = self.__kdv*(erreur - self.__previous_error_v)/self.__dt
        self.__previous_error_v = erreur
        # the final PID command
        comande = self.__proportional_v + self.__integral_v + self.__derivative_v
        # saturation with anti windup
        if comande > 0.8:
            comande = 0.8
            self.__integral_v = temp_integral
        elif comande < -0.8:
            comande = -0.8
            self.__integral_v = temp_integral
        return(comande)

    
    
    def pid_a(self, erreur):
        temp_integral_a = self.__integral_a
        self.__integral_a += erreur*self.__dt*self.__kia
        self.__proportional_a = self.__kpa*erreur
        self.__derivative_a = self.__kda*(erreur - self.__previous_error_a)/self.__dt
        self.__previous_error_a = erreur
        comande = self.__proportional_a + self.__integral_a + self.__derivative_a
        #saturation avec anti windup
        if comande > 1:
            comande = 1
            self.__integral_a = temp_integral_a
        elif comande < -1:
            comande = -1
            self.__integral_a = temp_integral_a
        return(comande)
    
    def stop(self):
        consigne = Twist()
        consigne.linear.x = 0
        consigne.linear.y = 0
        consigne.angular.z = 0
        self.pub.publish(consigne)
    
    def go_to(self):
        #on calcule a à partir de la position actuelle du robot
        err_x = self.go_x - self.x
        err_y = self.go_y - self.y
        abs_x = abs(err_x)
        abs_y = abs(err_y)
        angle = 0
        if err_x > 0.001 and err_y > 0:
            angle = math.atan2(abs_y,abs_x)
        elif err_x > 0.001 and err_y < 0:
            angle = -math.atan2(abs_y,abs_x)
        elif err_x < -0.001 and err_y > 0:
            angle = math.pi - math.atan2(abs_y,abs_x)
        elif err_x < -0.001 and err_y < 0:
            angle = math.pi + math.atan2(abs_y,abs_x)
        elif err_y > 0:
            angle = math.pi/2
        elif err_y < 0:
            angle = -math.pi/2
        angle = self.mod_2pi(angle)
        self.rotation(angle)
        self.translation(self.go_x, self.go_y)
        self.rotation(self.go_a)

    def go(self, rep):
        self.go_x = rep.x
        self.go_y = rep.y
        self.go_a = rep.z
        self.go_to()

pos = position()
rospy.spin()