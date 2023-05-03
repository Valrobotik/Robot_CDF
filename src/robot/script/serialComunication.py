#!/usr/bin/env python
import json
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
    def __init__(self, serial):
        Thread.__init__(self) #initialisation du thread
        self.__serial : MotSerial
        self.__serial = serial #serial port
        self.__left = 0 #vitesse roue gauche
        self.__right = 0 #vitesse roue droite
        return
    
    def run(self):
        rospy.Service('encoders', encoders, self.handle_encoders) #initialisation du service encoders
        self.getVitesse() #recuperation de la vitesse
        rospy.spin() #boucle infinie

    def getVitesse(self):
        rospy.sleep(0.1) #attente au demarrage
        rospy.loginfo("getVitesse-1")
        self.__serial.sendGcode("M403 \n") #envoie de la commande M403
        rospy.sleep(0.05) #attente de la reponse
        rospy.loginfo("getVitesse0")
        while True:
            rospy.loginfo("getVitesse")
            x = self.__serial.readline()#lecture de la reponse
            rospy.loginfo("getVitesse22")
            x = x.decode('utf8') 
            rospy.loginfo(x)
            data = x.replace('(', '').replace(')', '').split(';') #traitement de la reponse
            if len(data) == 5:
                self.__left = float(data[3]) #recuperation de la vitesse roue gauche
                self.__right = float(data[4]) #recuperation de la vitesse roue droite
            

    def handle_encoders(self, req):
        #on renvoie le position du client
        return encodersResponse(self.__left, self.__right) 

# thread d'envoie de la consigne de vitesse
class setVitConsignThread(Thread):
    def __init__(self, serial):
        Thread.__init__(self)
        self.__serial = serial

    def run(self):
        rospy.Subscriber("robot_consign", Twist, self.sendConsign)
        rospy.Subscriber("reset_all", Bool, self.reset)
        rospy.spin()

    def reset(self, msg:Bool):
        if msg.data:
            self.__serial.sendGcode("G26 X0 Y0 \n") 
        
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
        self.__serial.sendGcode(gcode)        
       
# ce thread permet d'envoyer des requetes de parametrage aux moteurs du robot
class requestMotorThread(Thread):
    def __init__(self, serial):
        Thread.__init__(self) # appel du constructeur du thread pour le lancer
        self.__serial = serial  # on recupere l'objet de communication serie du robot

    def run(self):
        # on ecoute le topic server_req pour recevoir les requetes du serveur sur la fonction sendReq
        rospy.Subscriber("server_req", String, self.sendReq)
        
        #on set les pid par defaut
        """rospy.sleep(0.1)
        gcode = "M301 P3.2 I0.5 D0.01\n"
        self.__serial.sendGcode(gcode)
        rospy.sleep(0.1)
        gcode = "M302 P8.5 I0.65 D0.01\n"
        self.__serial.sendGcode(gcode)
        #on attend les requetes"""
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
            rospy.loginfo("etat : " + str(self.busy()))
            if not(self.busy()): #on attend que le port soit libre
                self.setBusy() #on bloque le port
                self.write(gcode.encode("utf8")) #on envoie la commande
                rospy.loginfo("comande out : " + gcode) #on affiche la commande
                sended = True #on met l'etat de l'envoie a fait
                self.setUnbusy()# on debloque le port
                rospy.sleep(0.01)

    """def askVitesse(self):
        gcode = "M404 \n" #commande a envoyer
        sended = False # on initialise la variable qui permet de savoir si la commande a ete envoyee
        while not sended: # tant que la commande n'a pas ete envoyee
            if (not self.busy() and not self.__receipeserialBusy): # on verifie que le port serie n'est pas bloque
                self.setBusy() # on bloque le port serie pour eviter les conflits
                self.__receipeserialBusy = True # on bloque le port serie pour eviter les conflits
                #envoie de la commande
                self.write(gcode.encode("utf8")) # on envoie la commande recu en parametre
                rospy.sleep(0.005) #on attend 5ms pour eviter les bugs
                self.setUnbusy()#on libere le port serie
                print(gcode)    # on affiche la commande dans la console pour debug
                sended = True  # on indique que la commande a ete envoyee
                getit = True    # on initialise la variable qui permet de savoir si la reponse a ete recu
                rospy.sleep(0.005) # on attend 5ms pour eviter les bugs
                getime = time.time() # on recupere le temps actuel pour eviter les boucles infinies (Timeout)
                #reccuperation de la valeur de retour
                while getit and (time.time() - getime) < 0.05: # tant que la reponse n'a pas ete recu et que le timeout n'est pas atteint
                    sr = None # on initialise la variable qui contiendra la reponse
                    if self.inWaiting(): # on verifie que le port serie a recu quelque chose
                        by = self.readline() # on recupere la reponse
                        sr=by.decode('utf-8') # on decode la reponse
                        rospy.loginfo("recep : %s", sr)
                        getit = False # on verifie que la reponse n'est pas vide
                self.__receipeserialBusy = False # on debloque le port serie
        return sr   # on renvoie la reponse (sr est None si la reponse est vide ou si le timeout est atteint)"""


#ouverture de la connexion serie
serialName = rospy.get_param("motor_controller_port", "/dev/ttyACM0")
ser = MotSerial(serialName)

#lancement du noeud ROS : serialCon
rospy.init_node('serialCon', log_level=rospy.INFO)
rospy.loginfo("serialCon started")

# lancement des threads
posServer = getVitThread(ser)
posServer.start()

consServer = setVitConsignThread(ser)
consServer.start()

reqServer = requestMotorThread(ser)
reqServer.start()

rospy.spin() # on attend que le noeud soit termine
