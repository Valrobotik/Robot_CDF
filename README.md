# PAKAGE ROS ROBOT VALROB
Ce pakage est une verssion comenté du robot de la coupe de france merci de le garder fonctionel, toute modification doit etre commenter corectement. 

### LISTE DES NEUDS :
- ```serialCon``` situé dans le fichier : script/serialComunication.py
- ```imu``` situé dans le fichier : script/imu.py

### LISTE DES PUBLISHER/SUBSCRIBER :
- ```robot_consign``` type : Twist
  - subscribe (serialCon -> setVitConsigneThread.sendConsign())
- ```server_req``` type : String
  -  subscribe (serialCon -> requestMotorThread.sendReq)
- ```imu_data``` type : Twist
  - publish (imu)
### LISTE DES SERVICES :
- ```encoders``` type : encoders
  - service (SerialCon -> getVitThread.handle_encoders)

### LISTE DES PARAMETRE ROS : 
- ```motor_controller_port```



