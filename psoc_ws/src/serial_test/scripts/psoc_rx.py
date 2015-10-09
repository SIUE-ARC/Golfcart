#!/usr/bin/env python
import serial, rospy
from std_msgs.msg import String

def psoc_rx():
    pub = rospy.Publisher('psoc_rx', String)#, queue_size=1000) queue overflow. better to read
					    #rx buffer immediately.
    #initialize the serial port to read from a PSoC
    psoc = serial.Serial(port='/dev/ttyUSB2', baudrate=19200, stopbits=serial.STOPBITS_ONE, 
                        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, xonxoff=True,
                        rtscts=False, dsrdtr=False, timeout=0)
    rospy.init_node('psoc_rx', anonymous=True) #start the node
    r = rospy.Rate(60) #60hz publishing rate
    rcv = ""
    count = 0
    speed = 0
    angle = 0
    acpos = 0
    sspos = 0
    while not rospy.is_shutdown(): #run until program ended
        if psoc.inWaiting() >= 0:   #check for data in rx buffer
            rcv = psoc.read(psoc.inWaiting())  #read whole buffer
            #if rcv == 'd' or rcv == 'D':
             #   rcv = str(speed)
            #elif rcv == 's' or rcv == 'S':
             #   rcv = str(angle)
            #elif rcv == 'P' or rcv == 'p':
             #   rcv = str(sspos)
            #elif rcv == 'A' or rcv == 'a':
             #   rcv = str(acpos)
            str = rcv   #store buffer as publisher string
            pub.publish(str)   #publish over psoc_rx topic
            str = ''
        #flush all throughput streams
        psoc.flush()
        psoc.flushInput()
        psoc.flushOutput()
        r.sleep()

if __name__ == '__main__':
    try:
        psoc_rx()
    except rospy.ROSInterruptException: pass
