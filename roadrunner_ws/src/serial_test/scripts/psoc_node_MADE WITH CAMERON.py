#!/usr/bin/env python
import serial, rospy, time
from std_msgs.msg import String

#initialize the serial port parameters
psoc = serial.Serial()
psoc.baudrate = 19200
psoc.stopbits = serial.STOPBITS_ONE
psoc.bytesize = serial.EIGHTBITS
psoc.parity = serial.PARITY_NONE
psoc.timeout = 0
psoc.xonxoff = False
psoc.rtscts = False
psoc.dsrdtr = False

psoc.port = 0 #dont initialize port
tx_topic = ""
rx_topic = ""
cpd = 77.64232488822653 #counts per degree
ppms = 0 #pulses per ms 
sflag = False 
ppi = 0.0 #pulses per inch

velocity = 0.0
interval = 0.001 #1 msec wait for speed calculation
count = 0
old_count = 0
circumference = 91.89158511750145 #rear wheel cirumference
cmpc = 0.09189158511750145 #centimeters/count
#mphConv = cmpc * 22.373064458048 #mph conversion factor



#Function to initialize the communication between ROS and PSoC
def init_serial():    
    global psoc
    global rx_topic
    global tx_topic

# Jared is doing this...
    tty = "/dev/ttyUSB0"
    psoc.port = tty
    psoc.open() #open connection
    psoc.write("i\r") #send initialize command
    time.sleep(0.01)
    tx_topic = "drive_tx"
    rx_topic = "drive_rx"
    print "FOUND DRIVE\r"
    
    '''for i in range(0, 255): #loop over available ttyUSB ports and find the one the PSoC is using
        try:
            tty = "/dev/ttyUSB%d" % i
            psoc.port = tty
            if psoc.isOpen(): #tty port in use
                psoc.port = 0
                IOError("Port already open")
            else: #available port found
                psoc.open() #open connection
                psoc.write("i\r") #send initialize command
                time.sleep(0.01)
                data = psoc.read(psoc.inWaiting()) #check for response
                if str(data) is "S" or str(data) is 'S': #steer psoc responded
                    tx_topic = "steer_tx"
                    rx_topic = "steer_rx"
		    print "FOUND STEER\r"
                    break
                elif str(data) is "D" or str(data) is 'D': #drive psoc responded
                    tx_topic = "drive_tx"
                    rx_topic = "drive_rx"
		    print "FOUND DRIVE\r"
                    break
                else: #no psoc responded
                    psoc.close() #close connection
                    raise ValueError("No response")
        except serial.SerialException:
            print 'Port ttyUSB%d not available trying ttyUSB%d' % (i,i+1)
        except IOError:
            print "Port ttyUSB%d is in use! Trying ttyUSB%d..." % (i,i+1)
        except ValueError:
            print "Port ttyUSB%d did not respond to ping. Trying ttyUSB%d..." % (i,i+1)
    if not psoc.isOpen():
        print "No PSoC is connected"
        raise ValueError("No PSoC is connected")
'''
#convert angle to count
def convert_angle(angle):
    global cpd
    count = round(cpd*angle);
    return count
	
def convert_vel(vel):
    global old_count
    global count
    global velocity
    global circumference 
    return vel

#callback function is called when data is on the psoc_tx topic
def callback(data):
    global psoc
    angle = 0
    speed = 0
    command, blank, commandData = str(data).partition(" ")
    command, blank, param = str(commandData).partition(" ")
    if command is "t" or command is "T":
	angle = convert_angle(float(param))
	psoc.write(command)#writes data to PSoC
	psoc.write(angle)
    if command is "f" or command is "F" or command is "b" or command is "B":
        speed = convert_vel(param)
	dataToSend = command+blank+speed
	psoc.write(dataToSend)
	#psoc.write(command)
	#psoc.write(blank)
	#psoc.write(speed)
    '''if command is 'b' or command is 'B' or str(command) is "B" or str(command) is "b":
        speed = convert_vel(param)
	psoc.write(command)
	psoc.write(blank)
	psoc.write(speed) # <---Jared editted'''
    psoc.write("\r")#data must be terminated with a carriage return

def init_psoc():
    global rx_topic
    global tx_topic
    global psoc
    global velocity
    global interval
    global count
    global old_count
    init_serial()
    pub = rospy.Publisher(rx_topic, String)#, queue_size=1000) #queue overflow. better to read rx buffer immediately.
    rospy.init_node('psoc_node', anonymous=True)
    rospy.Subscriber(tx_topic, String, callback)#attach callback to subscriber
    r = rospy.Rate(60) #60hz publishing rate
    rcv = ""
    while not rospy.is_shutdown():
        #while not rospy.is_shutdown(): #run until program ended
        '''if psoc.inWaiting() > 0:   #check for data in rx buffer
            rcv = psoc.read(psoc.inWaiting())  #read whole buffer
            pub.publish(str(rcv))   #publish over psoc_rx topic
			#flush all throughput streams down the toilet
        psoc.write("D\r")
        time.sleep(interval)
        old_count = count
        count = psoc.read(psoc.inWaiting())
        velocity = (float(count) - float(old_count))/float(interval)   
        psoc.flush()
        psoc.flushInput()
        psoc.flushOutput()
        r.sleep()'''

   	if psoc.inWaiting() > 0:   #check for data in rx buffer
		rcv = psoc.read(psoc.inWaiting())  #read whole buffer
		pub.publish(str(rcv))   #publish over psoc_rx topic
	psoc.write("D\r")
	time.sleep(interval)
	old_count = count
	count = psoc.read(psoc.inWaiting())
	velocity = (float(count) - float(old_count))/float(interval)  #velocity in counts/msec
	#velocity = mphConv * velocity #converts velocity to mph
	print("Velocity in mph: " + str(velocity))
	psoc.flush()
	psoc.flushInput()
	psoc.flushOutput()
	r.sleep()

    #rospy.spin()
if __name__ == '__main__':
    try:
        init_psoc()
    except rospy.ROSInterruptException: pass
    except ValueError: pass

