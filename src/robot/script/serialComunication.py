#!/usr/bin/env python
from collections.abc import Callable, Iterable, Mapping
import json
from typing import Any
import serial
import rospy
from robot.srv import encoders, encodersResponse
from threading import Thread
from geometry_msgs.msg import Twist, Vector3
import sys
import time
from std_msgs.msg import String, Bool

    

# thread de recuperation de la vitesse lineaire et angulaire
class getVitThread(Thread): 
    def __init__(self, serial, moteur_send):
        Thread.__init__(self) #initialisation du thread
        self.__serial : MotSerial
        self.__sendgcode : sendGcodeThread = moteur_send
        self.__serial = serial #serial port
        self.__left = 0 #vitesse lineaire
        self.__right = 0 #vitesse angulaire
        return
    
    def run(self):
        rospy.Service('encoders', encoders, self.handle_encoders) #initialisation du service encoders
        self.getVitesse() #recuperation de la vitesse
        rospy.spin() #boucle infinie

    def getVitesse(self):
        rospy.sleep(1) #attente de la connection
        while True:
            self.__sendgcode.setM("M404 \n") #envoie de la commande M400
            timeout = time.time() + 0.3 #timeout de 300ms
            while self.__serial.in_waiting == 0 and timeout<time.time(): #tant que la reponse n'est pas recu on attend
                pass
            rospy.sleep(0.01) #attente de la reponse
            x = self.__serial.readline()#lecture de la reponse
            x = x.decode('utf8') 
            rospy.loginfo("comande in : " + x) #affichage de la reponse
            data = x.replace('R=(', '').replace(')', '').split(';') #traitement de la reponse
            if len(data) == 2: #si la reponse est correcte
                try : 
                    float(data[0]) #test de la validite de la reponse
                    float(data[1])
                    self.__left = float(data[0]) #recuperation de la vitesse roue gauche
                    self.__right = float(data[1]) #recuperation de la vitesse roue droite
                except ValueError:
                    pass
            

    def handle_encoders(self, req):
        #on renvoie le position du client
        return encodersResponse(self.__left, self.__right) 

# thread d'envoie de la consigne de vitesse
class setVitConsignThread(Thread):
    def __init__(self, serial):
        Thread.__init__(self)
        self.__serial : sendGcodeThread = serial

    def run(self):
        rospy.Subscriber("robot_consign", Twist, self.sendConsign)
        rospy.Subscriber("reset_all", Bool, self.reset)
        rospy.spin()

    def reset(self, msg:Bool):
        if msg.data:
            self.__serial.setG("G26 X0 Y0 \n") 
        
    def sendConsign(self, cons):
        gcode = ""
        if cons.angular.z == 0:
            gcode = "G26 X{0:.2f} Y{1:.2f} \n".format(cons.linear.x, cons.linear.y) 
        elif cons.angular.z == 1:
            gcode = "G11 I{0:.2f} J{1:.2f} \n".format(cons.linear.x, cons.linear.y)
        elif cons.angular.z == 2:
            gcode = "G10 I{0:.2f} J{1:.2f} \n".format(cons.linear.x, cons.angular.x)
        elif cons.angular.z == 3:
            gcode = "G13 I{0:.2f} J{1:.2f} \n".format(cons.linear.x, cons.angular.x)
        self.__serial.setG(gcode)
        
        
# ce thread permet d'envoyer des requetes de parametrage aux moteurs du robot
class requestMotorThread(Thread):
    def __init__(self, serial):
        Thread.__init__(self) # appel du constructeur du thread pour le lancer
        self.__serial = serial  # on recupere l'objet de communication serie du robot

    def run(self):
        # on ecoute le topic server_req pour recevoir les requetes du serveur sur la fonction sendReq
        rospy.Subscriber("server_req", String, self.sendReq)
        #on attend les requetes
        rospy.spin()

    def sendReq(self, req):
        """lorsque la fonction recoit une requete, elle la traite et envoie la commande au moteur"""
    
        #conversion en objet
        try:
            req = json.loads(req.data)
    
            gcode = ""
            #on prepare la commande en fonction de la requete
            if(req["type"] == "motor_request"):
                if(req["request"] == "set_pid_left"):
                    #M301 P.. I.. D..  => modifie les parametres PID du moteur gauche
                    gcode = "M301 P{0:.3f} I{1:.3f} D{2:.3f} \n".format(req["p"], req["i"], req["d"])
                elif(req["request"] == "set_pid_right"):
                    #M302 P.. I.. D..  => modifie les parametres PID du moteur droit
                    gcode = "M302 P{0:.3f} I{1:.3f} D{2:.3f} \n".format(req["p"], req["i"], req["d"])
                elif(req["request"] == "set_power_k"):
                    #M303 I.. J..  => modifie le gain de puissance du moteur gauche et droit
                    gcode = "M323 I{0:.3f} J{1:.3f} \n".format(req["l"], req["r"])
                elif(req["request"] == "set_measure_k"):
                    #M304 I.. J..  => modifie le gain de mesure du moteur gauche et droit
                    gcode = "M324 I{0:.3f} J{1:.3f} \n".format(req["l"], req["r"])
                #on envoie la commande M400 pour une raison inconnue à date du 18/04/2023
                self.__serial.sendGcode("M400 \n")    
            #on envoie la commande 
            self.__serial.sendGcode(gcode)  
        except Exception:
            pass

#element de communication serie avec le controleur moteur
class MotSerial(serial.Serial):
    def __init__(self, serialName):
        serial.Serial.__init__(self, serialName, 115200, timeout=0, dsrdtr=True) #initialisation de la connexion serie
        
        self.__sendserialBusy = False #etat de la connexion
        self.__receipeserialBusy = False #etat de la connexion

    def busy(self):
        return self.__sendserialBusy #renvoie l'etat de la connexion
    def setUnbusy(self):
        self.__sendserialBusy = False #met l'etat de la connexion a non occupe
    def setBusy(self):
        self.__sendserialBusy = True #met l'etat de la connexion a occupe

    def sendGcode(self, gcode):
        sended = False #etat de l'envoie
        while not sended: #tant que l'envoie n'est pas fait
            rospy.logdebug("etat : " + str(self.busy()))
            if not(self.busy()): #on attend que le port soit libre
                self.setBusy() #on bloque le port
                self.write(gcode.encode("utf8")) #on envoie la commande
                rospy.loginfo("comande out : " + gcode) #on affiche la commande
                sended = True #on met l'etat de l'envoie a fait
                self.setUnbusy()# on debloque le port
                rospy.sleep(0.02) #on attend 20ms

class sendGcodeThread(Thread):
    def __init__(self, serial):
        Thread.__init__(self)
        self.__serial : MotSerial = serial
        self.__G  = ""
        self.__M = ""
    
    def run(self):
        while not rospy.is_shutdown():
            if self.__G != "":
                g  = self.__G
                self.G = ""
                self.__serial.sendGcode(g)
            if self.__M != "":
                m = self.__M
                self.__M = ""
                self.__serial.sendGcode(m)
                
    def setG(self, gcode):
        self.__G = gcode
    def setM(self, gcode):
        self.__M = gcode
        
        
#ouverture de la connexion serie
serialName = rospy.get_param("motor_controller_port", "/dev/ttyACM0")
ser = MotSerial(serialName)

sendgcode = sendGcodeThread(ser)
sendgcode.start()

#lancement du noeud ROS : serialCon
rospy.init_node('serialCon', log_level=rospy.INFO)
rospy.logdebug("serialCon started")

# lancement des threads
posServer = getVitThread(sendgcode)
posServer.start()

consServer = setVitConsignThread(sendgcode)
consServer.start()

reqServer = requestMotorThread(ser)
reqServer.start()

rospy.spin() # on attend que le noeud soit termine
