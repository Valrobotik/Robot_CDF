# PAKAGE ROS ROBOT VALROB
Ce pakage est une verssion comenté du robot de la coupe de france merci de le garder fonctionel, toute modification doit etre commenter corectement. 

### LISTE DES NEUDS :
- ```serialCon``` situé dans le fichier : script/serialComunication.py
- ```imu``` situé dans le fichier : script/imu.py
- ```odometrie``` situé dans le fichier : script/odometrie.py
- ```asserv``` situé dans le fichier : script/asserv.py
- ```match``` situé dans le fichier : script/strat.py

### LISTE DES PUBLISHER/SUBSCRIBER :
- ```robot_consign``` type : Twist
  - publish (asserv)
  - subscribe (serialCon -> setVitConsigneThread.sendConsign())
- ```server_req``` type : String
  -  subscribe (serialCon -> requestMotorThread.sendReq())
- ```imu_data``` type : Imu
  - publish (imu)
- ```Odom``` type : Odometrie
  - publish (odometrie)
  - subscribe (asserv -> position.odom())
- ```go``` type : Vector3
  - publish (match)
  - subscribe (asserv -> position.go())
- ```reset_all``` type : Bool
  - publish (match)
  - subscribe (serialCon -> setVitConsignThread.reset())


### LISTE DES SERVICES :
- ```encoders``` type : encoders.srv (situé dans le dossier srv)
  - service (SerialCon -> getVitThread.handle_encoders)
  - client (odometrie)

### LISTE DES PARAMETRE ROS : 
- ```motor_controller_port```



