roslaunch diff_GPS diff_GPS.launch 

baudrate 460800
choose low number uart device

using rtk_corrections package in common on jackal setup a reciever node that reads the rtk corrections transmitted 
by the base station and publishes them on a ros topic, then have your diff gps node read the data from this ros topic 
and send it to the diff_gps over serial. then test outside
