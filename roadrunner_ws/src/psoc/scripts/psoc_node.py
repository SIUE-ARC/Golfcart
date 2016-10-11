#!/usr/bin/env python
import serial, rospy, time
from std_msgs.msg import String

#initialize the serial port parameters
psoc_drive = serial.Serial()
psoc_drive.baudrate = 19200
psoc_drive.stopbits = serial.STOPBITS_ONE
psoc_drive.bytesize = serial.EIGHTBITS
psoc_drive.parity = serial.PARITY_NONE
psoc_drive.timeout = 0
psoc_drive.xonxoff = False
psoc_drive.rtscts = False
psoc_drive.dsrdtr = False

psoc_drive.port = 0 #dont initialize port
drive_tx_topic = "drive_tx"
drive_rx_topic = "drive_rx"

psoc_steer = serial.Serial()
psoc_steer.baudrate = 19200
psoc_steer.stopbits = serial.STOPBITS_ONE
psoc_steer.bytesize = serial.EIGHTBITS
psoc_steer.parity = serial.PARITY_NONE
psoc_steer.timeout = 0
psoc_steer.xonxoff = False
psoc_steer.rtscts = False
psoc_steer.dsrdtr = False

psoc_steer.port = 0 #dont initialize port
steer_tx_topic = "steer_tx"
steer_rx_topic = "steer_rx"

cpd = 77.64232488822653 #counts per degree
ppms = 0 #pulses per ms
sflag = False
ppi = 0.0 #pulses per inch

velocity = 0.0
interval = 0.01
count_drive = 0
old_count_drive = 0
count_steer = 0
old_count_steer = 0
circumference = 91.89158511750145
cmpc = 0.09189158511750145 #centimeters/count
#mphConv = cmpc * 22.373064458048 #mph conversion factor
mphConv = 2.0558963569861

#Function to initialize the communication between ROS and PSoC
def init_serial(serial_port, psoc_type):
    for i in range(0, 255): #loop over available ttyUSB ports and find the one the PSoC is using
        try:
            tty = "/dev/ttyUSB%d" % i
            serial_port.port = tty
            if serial_port.isOpen(): #tty port in use
                serial_port.port = 0
                IOError("Port already open")
            else: #available port found
                serial_port.open() #open connection
                serial_port.write("i\r") #send initialize command
                time.sleep(0.01)
                data = serial_port.read(serial_port.inWaiting()) #check for response
                if str(data) is "S" or str(data) is 'S': #steer psoc responded
                    if psoc_type is "S":
                        print "FOUND STEER\r"
                        break
                    else:
                        serial_port.close()
                        raise ValueError("No response")
                elif str(data) is "D" or str(data) is 'D': #drive psoc responded
                    if psoc_type is "D":
                        print "FOUND DRIVE\r"
                        break
                    else:
                        serial_port.close()
                        raise ValueError("No response")
                else: #no psoc responded
                    serial_port.close() #close connection
                    raise ValueError("No response")
        except serial.SerialException:
            print 'Port ttyUSB%d not available trying ttyUSB%d' % (i,i+1)
        except IOError:
            print "Port ttyUSB%d is in use! Trying ttyUSB%d..." % (i,i+1)
        except ValueError:
            print "Port ttyUSB%d did not respond to ping. Trying ttyUSB%d..." % (i,i+1)
    if not serial_port.isOpen():
        print "No PSoC is connected"
        serial_port.close()
        raise ValueError("No PSoC is connected")

#convert angle to count
def convert_angle(angle):
    global cpd
    count = round(cpd*angle)
    return str(count) #JC - changed from count to str(count) 11/13/15

def convert_vel(vel):
    global old_count_drive
    global count_drive
    global velocity
    global circumference
    return vel


#callback function is called when data is on the drive_tx topic
def drive_interpreter(data):
    global psoc_drive
    angle = 0
    speed = 0
    command, blank, commandData = str(data).partition(" ")
    command, blank, param = str(commandData).partition(" ")
    if command is "t" or command is "T":
	angle = convert_angle(float(param))
	psoc_drive.write(command)#writes data to PSoC
	psoc_drive.write(blank)#JC - added this line 11/13/15
	psoc_drive.write(angle)
    if command is "f" or command is "F":
        speed = convert_vel(param)
	#dataToSend = command+blank+speed
	#psoc_drive.write(dataToSend)
	psoc_drive.write(command)
	psoc_drive.write(blank)
	psoc_drive.write(speed)
    if command is 'b' or command is 'B' or str(command) is "B" or str(command) is "b":
        speed = convert_vel(param)
	psoc_drive.write(command)
	psoc_drive.write(blank)
	psoc_drive.write(speed) # <---Jared editted'''
    psoc_drive.write("\r")#data must be terminated with a carriage return

#callback function is called when data is on the steer_tx topic
def steer_interpreter(data):
    global psoc_steer
    angle = 0
    speed = 0
    command, blank, commandData = str(data).partition(" ")
    command, blank, param = str(commandData).partition(" ")
    if command is "t" or command is "T":
	angle = convert_angle(float(param))
	psoc_steer.write(command)#writes data to PSoC
	psoc_steer.write(blank)#JC - added this line 11/13/15
	psoc_steer.write(angle)
    psoc_steer.write("\r")
    if command is "f" or command is "F":
        speed = convert_vel(param)
	#dataToSend = command+blank+speed
	#psoc_steer.write(dataToSend)
	psoc_steer.write(command)
	psoc_steer.write(blank)
	psoc_steer.write(speed)
    if command is 'b' or command is 'B' or str(command) is "B" or str(command) is "b":
        speed = convert_vel(param)
	psoc_steer.write(command)
	psoc_steer.write(blank)
	psoc_steer.write(speed) # <---Jared editted'''
    psoc_steer.write("\r")#data must be terminated with a carriage return

def init_psoc():
    global drive_rx_topic
    global drive_tx_topic
    global steer_rx_topic
    global steer_tx_topic
    global psoc_drive
    global psoc_steer
    global velocity
    global interval
    global count_drive
    global old_count_drive
    global count_steer
    global old_count_steer
    global mphConv
    global cmpc

    init_serial(psoc_drive, "D")
    #time.sleep(.1)
    init_serial(psoc_steer, "S")
    #create publisher for drive and steer PSoCs
    pub_drive = rospy.Publisher(drive_rx_topic, String)
    pub_steer = rospy.Publisher(steer_rx_topic, String)
    rospy.init_node('psoc_node', anonymous=True)
    #attach callbacks to subscribers
    drive_sub = rospy.Subscriber(drive_tx_topic, String, drive_interpreter)
    steer_sub = rospy.Subscriber(steer_tx_topic, String, steer_interpreter)
    r = rospy.Rate(5) #40hz publishing rate
    steer_rcv = ""
    drive_rcv = ""
    #print("Starting node...")
    while not rospy.is_shutdown():
        #print("Node started!")
        #while not rospy.is_shutdown(): #run until program ended
       	if psoc_steer.inWaiting() > 0:   #check for data in rx buffer
            #print("Got %d bytes from steer" % psoc_steer.inWaiting())
            steer_rcv = psoc_steer.read(psoc_steer.inWaiting())  #read whole buffer
            pub_steer.publish(str(steer_rcv))   #publish over psoc_steer_rx topic
        if psoc_drive.inWaiting() > 0:
            #print("Got %d bytes from steer" % psoc_drive.inWaiting())
            drive_rcv = psoc_drive.read(psoc_drive.inWaiting())
            pub_drive.publish(str(drive_rcv))
        #print("Sending stuff")
        #if tx_topic == "drive_tx":
        '''psoc_drive.write("D\r")
        time.sleep(interval)
        old_count_drive = count_drive
        print("Old Count is: " + str(old_count_drive))
        count_drive = psoc_drive.read(psoc_drive.inWaiting())
        print("New Count is: " + str(count_drive))
        velocity = (float(count_drive) - float(old_count_drive))/float(interval*1000)
        print("Velocity is: " + str(velocity))
        milesPerHour = float(mphConv) * float(velocity) #converts velocity to mph
        print("Velocity in mph: " + str(milesPerHour))
        #if tx_topic == "steer_tx":
        time.sleep(.1)
        psoc_steer.write("S\r")
        time.sleep(interval)
        old_count_steer = count_steer
        print("Steer Count is: " + str(old_count_steer))
        count_steer = psoc_steer.read(psoc_steer.inWaiting())
        print("Steer Count is: " + str(count_steer))
        velocity = (float(count_steer) - float(old_count_steer))/float(interval*1000)
        print("Steer Velocity is: " + str(velocity))
        #JC - commented this ^ section on 11/13/15'''
        psoc_steer.flush()
        psoc_steer.flushInput()
        psoc_steer.flushOutput()
        psoc_drive.flush()
        psoc_drive.flushInput()
        psoc_drive.flushOutput()
        r.sleep()
    #rospy.spin()
if __name__ == '__main__':
    try:
        init_psoc()
    except rospy.ROSInterruptException:
        psoc_steer.close()
        psoc_drive.close()
    except ValueError:
        psoc_steer.close()
        psoc_drive.close()
