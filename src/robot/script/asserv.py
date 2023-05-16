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
        """initialisation du noeud asservissement"""

        #initialisation du noeud
        rospy.init_node("asserv", log_level=rospy.DEBUG)
        
        #constantes du PID lineaire
        self.__kpv = 0.6
        self.__kiv = 0.00001
        self.__kdv = 0
        
        #constantes du PID angulaire
        self.__kpa = 5
        self.__kia = 0.1
        self.__kda = 0
        
        #erreurs lineaire et angulaire tolerees pour la fin du PID
        self.error_l = 0.01
        self.error_a = 0.05

        #frequence d'asservissement
        self.__freq_aserv = 50
        
        #variables du PID lineaire
        self.__integral_v = 0
        self.__integral_a = 0
        
        self.__previous_error_v = 0
        self.__previous_error_a = 0
        
        self.__dt = 0
        
        #etat du robot
        self.__action = True
        
        #position du robot
        self.x = 0
        self.y = 0
        self.a = 0
        
        #consigne du robot
        self.go_a = 0
        self.go_x = 0
        self.go_y = 0
        
        self.__list_ordre = []

        #initialisation des subscribers et publishers
        rospy.Subscriber("/Odom", Odometry, self.odom)
        self.pub = rospy.Publisher("robot_consign", Twist, queue_size=10)
        rospy.Subscriber("go", Vector3, self.go)
        rospy.Subscriber("reset_all", Bool, self.reset)
        rospy.Subscriber("break", Bool, self.break_)
   
    def reset(self, rep:Bool):
        """reset des variables du PID"""
        if rep.data:
            self.__integral_v = 0
            self.__integral_a = 0  
            self.__previous_error_v = 0
            self.__previous_error_a = 0
            self.__dt = 0
            self.__action = True
            
            #position du robot
            self.x = 0
            self.y = 0
            self.a = 0

    def break_(self, rep:Bool):
        """arret du robot"""
        if rep.data:
            self.__list_ordre = []
            self.__action = False
            
    def odom(self, rep):
        """recuperation de la position du robot"""
        self.x = rep.pose.pose.position.x
        self.y = rep.pose.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion([rep.pose.pose.orientation.x, rep.pose.pose.orientation.y, rep.pose.pose.orientation.z, rep.pose.pose.orientation.w])
        self.a = self.mod_2pi(yaw) #angle entre -pi et pi
        
    def rotation(self, angle):
        """rotation du robot vers l'angle voulu (position absolue en radian)"""

        #initialisation de la consigne
        consigne = Twist()
        consigne.linear.x = 0
        consigne.linear.y = 0
        consigne.angular.z = 3
        self.__integral_a = 0
        self.__previous_error_a  = self.mod_2pi(angle - self.a)
        previous_time = time.time()
        rate = rospy.Rate(self.__freq_aserv)
        while abs(self.mod_2pi(angle - self.a)) > self.error_a and self.__action:
            #boucle d'asservissement
            self.__dt = time.time() - previous_time
            previous_time = time.time()
            consigne.angular.x = self.pid_a(self.mod_2pi(angle - self.a)) #calcul de la consigne angulaire
            self.pub.publish(consigne) #publication de la consigne
            rate.sleep() #attente de la boucle de publication à 50Hz 
    
    def translation(self, x, y):
        """deplacement du robot vers le point voulu (position absolue (x,y) en m)"""
        #initialisation de la consigne
        consigne = Twist()
        consigne.linear.y = 0
        consigne.angular.z = 3
        previous_time = time.time()
        rate = rospy.Rate(self.__freq_aserv)
        while (abs(self.x - x) > self.error_l or abs(self.y - y) > self.error_l) and self.__action:
            #boucle d'asservissement
            self.__dt = time.time() - previous_time
            previous_time = time.time()

            #calcul des erreur en x et y
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
            err_a = self.mod_2pi(angle - self.a)

            #on verifie que l'angle a atteindre n'est pas trop grand (sinon on fais marche arriere)
            if abs(err_a) > math.pi/2:
                err_a = self.mod_2pi(err_a + math.pi)
                signe = -1
            else:
                signe = 1
            
            #calcul de la consigne
            consigne.angular.z = self.pid_a(err_a)
            consigne.linear.x = self.pid_v(signe*math.sqrt(err_x**2 + err_y**2))

            self.pub.publish(consigne) #publication de la consigne
            rate.sleep() #attente de la boucle de publication à 50Hz
        
    def mod_2pi(self, angle):
        """fonction qui ramene un angle entre -pi et pi"""
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return(angle)
        
    def pid_v(self, erreur):
        """PID en vitesse"""
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
        """PID en angle"""
        #terme integral du PID
        temp_integral_a = self.__integral_a
        self.__integral_a += erreur*self.__dt*self.__kia
        #terme proportionnel du PID
        self.__proportional_a = self.__kpa*erreur
        self.__derivative_a = self.__kda*(erreur - self.__previous_error_a)/self.__dt
        self.__previous_error_a = erreur
        #comande finale du PID
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
        """arret du robot"""
        consigne = Twist()
        consigne.linear.x = 0
        consigne.linear.y = 0
        consigne.angular.z = 0
        self.pub.publish(consigne) #publication de la consigne
    
    def go_to(self):
        """on se deplace vers le point voulu (position relative (x,y, theta) en m x m x rad)"""
        #calcule de l'angle a atteindre en fonction de la position du point voulu
        #on cherche à pouvoir se deplacer en ligne droite
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

        #on initialise une rotation pour se mettre dans la bonne direction
        rospy.logdebug("go angle : %f", angle)
        self.rotation(angle)
        #on avance jusqu'au point voulu
        rospy.logdebug("go distance : %f", math.sqrt(err_x**2 + err_y**2))
        self.translation(self.go_x, self.go_y)
        if self.go_a != None: #si on a une orientation final a atteindre on l'atteint
            self.rotation(self.go_a)
        self.stop()

    def go(self, rep):
        """on reçoit un ordre de deplacement (position relative (x,y, theta) en m x m x rad) via le topic /go_to"""
        while not self.__action: pass #on attend en cas de break que l'action ai a nouveau le droit d'etre effectuee (pour eviter les conflits)
        self.__list_ordre.append(rep) #on ajoute l'ordre a la liste
    
    def go_to_list(self):
        """on se deplace vers les points de la liste successivement"""
        while not rospy.is_shutdown():
            if len(self.__list_ordre) > 0: #si il y a des ordres dans la liste on les execute
                self.go_x = self.__list_ordre[0].x
                self.go_y = self.__list_ordre[0].y
                self.go_a = self.__list_ordre[0].z
                self.go_to() #on se deplace vers le point
                self.__list_ordre.pop(0) #on supprime l'ordre de la liste
            else: #sinon on s'arrete
                self.stop() #on s'arrete
                self.__action = True #on autorise les nouvelles actions dans le cas d'un break
                
pos = position()
pos.go_to_list()