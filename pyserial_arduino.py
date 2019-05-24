
#!/usr/bin/env python

import roslib
import rospy
import math 
import time
import rospy
import numpy as np
import time 
import serial 
import os

from std_msgs.msg import Char 
from std_msgs.msg import Int8

msg=Char()
pos=Int8()
past_pos=1
pos_char=0
publisher_pos = rospy.Publisher('servo_pos', Int8, queue_size=1)

def callback(data):	
    global msg
    msg=data

def setup():
    #Ros susbscriber B ES EL ROJOOOOOO
    rospy.init_node('serial_com')
    rospy.Subscriber("servo_move", Char, callback)
    pos.data=0
    msg.data=0x00
    publisher_pos.publish(pos)


if __name__=="__main__":
    aux=0x02	
    setup()

    r = rospy.Rate(10)
    print 'init'
    os.system('sudo chmod 777 /dev/ttyACM2') 
    arduino=serial.Serial('/dev/ttyACM2', 115200, timeout=0.1)
    time.sleep(0.5)
    print 'done'
    while not rospy.is_shutdown():
        try:
	    
            if (msg.data != aux): #update only if message is new
                try:
                    aux=msg.data
                    arduino.write(chr(msg.data)) #NECESSARY TO SEND AS CHAR
                    print 'Send Byte: '+str(msg)
                    time.sleep(0.01)
                except:
                    print 'failed serial com sending'
                    pass

	    if ( (aux & 0x0F) != 0x00): #ask arduino for info
	        try:
	            request=msg.data | 0x40
	            arduino.write(chr(request))
		    print 'Send Req Byte: '+str(request)
		    time.sleep(0.01) 
	            while arduino.in_waiting==0:  
	                pass
		    servo=arduino.read()
		    #print servo 
	            pos.data = ord(servo)-90
		    print 'ya guarde el dat0'
	            publisher_pos.publish(pos)
	            #print 'Received Pos: '+str(pos.data)

	        except:
	            print 'failed serial com receiving' 

        except Exception as e:
                print (e)
                pass

#!/usr/bin/env python
