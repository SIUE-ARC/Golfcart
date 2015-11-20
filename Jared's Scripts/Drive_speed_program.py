# Node based drive commands
# Node based steer commands
# 5/29/15


import serial
import time
from time import sleep # "sleep(seconds)"

driveport = serial.Serial("/dev/ttyUSB0", 19200, timeout = 0)
steerport = serial.Serial("/dev/ttyUSB1", 19200, timeout = 0)
serial.EIGHTBITS
serial.PARITY_NONE
serial.STOPBITS_ONE


steerport.write('i\r') #for packet serial baud rate
#driveport.write('i\r') #
sleep(2)
steerport.write('r\r') #reset
driveport.write('a\r')
sleep(0.5)
wait = 15


driveport.write('f 255\r') #forward
sleep(3)
#driveport.flus1hInput()
print("Starting!\r")
driveport.write('d\r') #get encoder count
#blank = driveport.read(100)
#start = driveport.read(100) #
#print(start)
sleep(wait)

#driveport.flushInput()
driveport.write('d\r') # get encoder count again
print("Done!\r")
sleep(2)
driveport.write('f 0\r') #stop
sleep(2)
stop = driveport.read(1000) #
print(stop)


sleep(4)
driveport.write('b 255\r')
sleep(wait+6)
driveport.write('f 0\r')
sleep(0.5)
driveport.write('a\r')
print("BACK HOMEISH!")




#strstart = str(start)
#strstop = str(stop)
#strwait = str(wait)

#print("Start is: " + strstart + "\n")
#print("Stop is: " + strstop + "\n")
#print("Time between: " + strwait + "\n")

