#!/usr/bin/env python

# -*- coding: utf-8 -*-
# Path: Valrob_pkg\script\serialComunication.py
# envoie et reception de commande sur le port serie avec la carte de controle des moteurs du robot

import json
import serial
import rospy
from robot.srv import encoders, encodersResponse
from threading import Thread 
from geometry_msgs.msg import Twist, Vector3
import time
from std_msgs.msg import String

    

#thread qui permet de gerer les requetes de vitesse des encodeurs du robot
class getVitThread(Thread):
    def __init__(self, serial):
        Thread.__init__(self) # appel du constructeur du thread pour le lancer
        self.__serial = serial # on recupere l'objet de communication serie du robot
        return
    def run(self):
        
        rospy.Service('encoders', encoders, self.handle_encoders) # on ecoute les apelle au service encoders pour recevoir les requetes du serveur sur la fonction handle_encoders
        rospy.spin() # on attend les requetes

    def getVitesse(self):
        resp = self.__serial.sendWithResponse("M404 \n") # on envoie la commande M404 de recuperation des encodeurs
        if resp == None : resp = "enc=(-1;-1)" # si la communication echoue on renvoie la valeur par defaut   
        return resp 

    def handle_encoders(self, req):
        strData = self.getVitesse()  # on recupere la reponse du robot en string
        data = strData.replace('enc=(', '').replace(')', '').split(';') # on la transforme en tableau de float
        return encodersResponse(float(data[0]),float(data[1]))  


class setVitConsignThread(Thread):
    def __init__(self, serial):
        Thread.__init__(self)
        self.__serial = serial

    def run(self):
        rospy.Subscriber("robot_consign", Twist, self.sendConsign)
        rospy.spin()

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

# cette objet serie permet de gerer les requettes sur le port serie
class MotSerial(serial.Serial):
    def __init__(self, serialName):
        #on ouvre le port serie passer en parametre avec une vitesse de 115200 bauds
        serial.Serial.__init__(self, serialName, 115200, timeout=0)
        self.__serialBusy = False # on initialise le port serie comme libre
        '''INFO : le port serie est bloqué quand il est en train d'envoyer une commande ou de recevoir une reponse de la part du robot
            il est donc impossible d'envoyer une commande ou de recevoir une reponse tant que le port serie est bloqué'''
    
    def busy(self):
        return self.__serialBusy # on renvoie l'etat du port serie
    
    def setUnbusy(self):
        self.__serialBusy = False # on libere le port serie
        
    def setBusy(self):
        self.__serialBusy = True # on bloque le port serie
        
    def sendGcode(self, gcode):
        """on envoie une commande au robot en format gcode"""
        sended = False # on initialise la variable qui permet de savoir si la commande a ete envoyee
        while not sended: # tant que la commande n'a pas ete envoyee
            if (not self.busy()): # on verifie que le port serie n'est pas bloque 
                self.setBusy() # on bloque le port serie pour eviter les conflits
                time.sleep(0.01) # on attend un peut **(pourquoi ici ?)**
                self.write(gcode.encode("utf8")) # on envoie la commande recu en parametre
                print(gcode) # on affiche la commande dans la console pour debug
                sended = True   # on indique que la commande a ete envoyee
                self.setUnbusy()# on libere le port serie
    
    def sendWithResponse(self, gcode):
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
                time.sleep(0.01) # on attend un peut pour laisser le temps au robot de repondre
                #reccuperation de la valeur de retour
                while getit and (time.time() - getime) < 0.1: # tant que la reponse n'a pas ete recu et que le timeout n'est pas atteint
                    sr = None # on initialise la variable qui contiendra la reponse
                    if self.inWaiting(): # on verifie que le port serie a recu quelque chose
                        by = self.readline() # on recupere la reponse
                        sr=by.decode('utf-8') # on decode la reponse
                        getit = (sr == None) # on verifie que la reponse n'est pas vide
                self.setUnbusy()#on libere le port serie
        return sr   # on renvoie la reponse (sr est None si la reponse est vide ou si le timeout est atteint)

serialName = rospy.get_param("motor_controller_port", "/dev/ttyACM0") # on recupere le nom du port serie definit avec les parametres du robot

ser = MotSerial(serialName) # on initialise l'objet serie avec le nom du port serie

#execution server position
rospy.init_node('serialCon') # on initialise le noeud ROS

vitServer = getVitThread(ser) # on initialise le thread qui gere les demandes de vitesse
vitServer.start() # on lance le thread

consServer = setVitConsignThread(ser) # on initialise le thread qui gere les requetes de consigne de vitesse
consServer.start() # on lance le thread

reqServer = requestMotorThread(ser) # on initialise le thread qui gere les requetes de parametres du moteur
reqServer.start() # on lance le thread