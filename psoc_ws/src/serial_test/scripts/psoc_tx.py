#!/usr/bin/env python
import serial, rospy
from std_msgs.msg import String

tty0 = "/dev/ttyUSB0" #make sure this matches the port the PSoC is connected to!!!
baud = 19200
#initialize the serial port for the PSoC
psoc = serial.Serial(port='/dev/ttyUSB2', baudrate=baud, stopbits=serial.STOPBITS_ONE, 
                        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, xonxoff=True,
                        rtscts=False, dsrdtr=False, timeout=0, writeTimeout=0.01)
#callback function is called when data is on the psoc_tx topic
def callback(data):
    #msg = int(data.data)
    #msg = bytes(msg)
    psoc.write(data.data)#writes data to PSoC
    #psoc.write("\r")#data must be terminated with a carriage return

#def convert_angle(angle):

#def convert_vel(vel):

    
def psoc_tx():
    rospy.init_node('psoc_tx', anonymous=True)#start node

    rospy.Subscriber("psoc_tx", String, callback)#attach callback to subscriber

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    psoc_tx()
