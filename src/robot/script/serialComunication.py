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
        self.__serial = serial #serial port
        return
    
    def run(self):
        rospy.Service('encoders', encoders, self.handle_encoders) #initialisation du service encoders
        rospy.spin() #boucle infinie

    def getVitesse(self):
        resp = self.__serial.askVitesse() # on demande la vitesse au robot
        if resp == None : resp = "R=(-1;-1)" # si la communication echoue on renvoie la valeur par defaut   
        return resp 

    def handle_encoders(self, req):
        #on renvoie le position du client
        strData = self.getVitesse()
        data = strData.replace('R=(', '').replace(')', '').split(';')
        rospy.loginfo("vitesse : " + str(data))
        return encodersResponse(float(data[0]),float(data[1])) 

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
        serial.Serial.__init__(self, serialName, 115200, timeout=0) #initialisation de la connexion serie
        self.__serialBusy = False #etat de la connexion
    def busy(self):
        return self.__serialBusy #renvoie l'etat de la connexion
    def setUnbusy(self):
        self.__serialBusy = False #met l'etat de la connexion a non occupe
    def setBusy(self):
        self.__serialBusy = True #met l'etat de la connexion a occupe
    def sendGcode(self, gcode):
        sended = False #etat de l'envoie
        while not sended: #tant que l'envoie n'est pas fait
            if (not self.busy()): #on attend que le port soit libre
                self.setBusy() #on bloque le port
                self.write(gcode.encode("utf8")) #on envoie la commande
                rospy.loginfo("comande out : " + gcode) #on affiche la commande
                sended = True #on met l'etat de l'envoie a fait
                self.setUnbusy()# on debloque le port
                
    def askVitesse(self):
        gcode = "M404 \n" #commande a envoyer
        sended = False # on initialise la variable qui permet de savoir si la commande a ete envoyee
        while not sended: # tant que la commande n'a pas ete envoyee
            if (not self.busy()): # on verifie que le port serie n'est pas bloque
                self.setBusy() # on bloque le port serie pour eviter les conflits
                #envoie de la commande
                self.write(gcode.encode("utf8")) # on envoie la commande recu en parametre
                print(gcode)    # on affiche la commande dans la console pour debug
                sended = True  # on indique que la commande a ete envoyee
                getit = True    # on initialise la variable qui permet de savoir si la reponse a ete recu
                getime = time.time() # on recupere le temps actuel pour eviter les boucles infinies (Timeout)
                #reccuperation de la valeur de retour
                rospy.sleep(0.01) # on attend 10ms pour eviter les bugs
                while getit and (time.time() - getime) < 0.05: # tant que la reponse n'a pas ete recu et que le timeout n'est pas atteint
                    sr = None # on initialise la variable qui contiendra la reponse
                    if self.inWaiting(): # on verifie que le port serie a recu quelque chose
                        by = self.readline() # on recupere la reponse
                        sr=by.decode('utf-8') # on decode la reponse
                        rospy.loginfo("recep : %s", sr)
                        getit = False # on verifie que la reponse n'est pas vide
                self.setUnbusy()#on libere le port serie
        return sr   # on renvoie la reponse (sr est None si la reponse est vide ou si le timeout est atteint)


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
