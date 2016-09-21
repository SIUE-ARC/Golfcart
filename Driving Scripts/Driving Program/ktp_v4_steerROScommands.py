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

speed = 80 #forward speed
bspeed = 90 #backup speed
strspeed = str(speed)
bstrspeed = str(bspeed)
EBRAKE = 0 #ebrake flag
AUTO = 0 #autonomous flag

print("Enter Key:\n\n\r '0' to stop all motors,'a' for left,\n\r 'd' for right,\n\r 'l' to continue straight,\n\r 'w' to increase forward speed(+10),\n\r 's' to decrease forward speed(-10),\n\r 'B' to increase backwards speed(+5),\n\r 'b' to decrease backwards speed(-5),\n\r 'u' to toggle E-Brake use,\n\r 'h on' to engage brake,\n\r 'h off' to disengage brake,\n\r 'auto' to toggle simulated autonomy,\n\r 'senc' for steer encoder,\n\r 'denc' for drive encoder,\n\r 'exit' to exit program,\n\n\r Forward Speed = " + strspeed + "\n\r Backwards Speed = "  + bstrspeed + "\n\r")

while True:

	key = raw_input("Enter a,d,l,w,s, 0 to STOP, or 'dir'> ")

	if key== "w":
		speed = speed + 10
		strspeed = str(speed)
		driveport.write('f ') #send forward
		driveport.write(strspeed) #send value to drive motor controller
		driveport.write('\r')
		print("New Speed is: "+ strspeed)
		
	elif key== "s":
		speed = speed - 10
		strspeed = str(speed)
		driveport.write('f ') #send forward
		driveport.write(strspeed) #send value to drive motor controller
		driveport.write('\r')
		print("New Speed is: "+ strspeed)

	elif key== "B":
		bspeed = bspeed + 5
		bstrspeed = str(bspeed)
		driveport.write('b ') #send backward
		driveport.write(bstrspeed) #send value to drive motor controller
		driveport.write('\r')
		print("New backward speed is: "+ bstrspeed)

	elif key== "b":
		bspeed = bspeed - 5
		bstrspeed = str(bspeed)
		driveport.write('b ') #send backward
		driveport.write(bstrspeed) #send value to drive motor controller
		driveport.write('\r')
		print("New backward speed is: "+ bstrspeed)

	elif key=="a": #turn left
		steerport.write('q\r') #hex left >d'80 (128,0,80,80)

	elif key=="d": #turn right
		steerport.write('e\r') #hex right >d'80 (128,0,80,80)

	elif key=="l": # go straight
		steerport.write('l\r') #steer motor stop (128,0,0,0)
	
	elif key=="u": # ESTOP Brake enable/disable
		steerport.write('u\r') #useestobrake
		if EBRAKE == 0:
			EBRAKE = 1
			print("E-Brake on")
		elif EBRAKE == 1:
			EBRAKE = 0
			print("E-Brake off")
	
	elif key=="i": # baud
		steerport.write('i\r') #baud char
		print("Baud Character Sent")

	elif key=="h on": # brake on
		steerport.write('h on\r') #baud char

	elif key=="h off": # baud
		steerport.write('h off\r') #baud char

	elif key=="dtest": # drive test
		driveport.write('f 90\r') #forward 
		sleep(3)
		driveport.write('f 0\r') #stop
		steerport.write('h on\r') #brake on
		sleep(4)
		steerport.write('h off\r') #brake off
		sleep(4)
		driveport.write('b 90\r') #backwards
		sleep(3)
		driveport.write('b 0\r') #stop
		steerport.write('h on\r') #brake on
		sleep(4)
		steerport.write('h off\r') #brake off
		sleep(4)
		driveport.write('f 0\r') #release relay		

	elif key=="0": #stop motors
		driveport.write('f ') #send forward
		driveport.write("0")
		driveport.write('\r')
		steerport.write('l\r')
		speed = 80
		bspeed = 90
		print("Motors stopped")
	elif key=="r": #reset steer shaft
		steerport.write('r\r')

	elif key=="auto": # flash safety light toggle
		driveport.write('a\r') #autonomous modesenc
		if AUTO == 0:
			AUTO = 1
			print("AUTONOMOUS MODE")
		elif AUTO == 1:
			AUTO = 0
			print("MANUAL MODE")

	elif key=="senc": # request steer encoder count
		steerport.write('s\r')
		senc = steerport.read(100)
		#senc = steerport.readline() #autonomous mode
		print(senc+"\n")

	elif key=="denc": # request drive encoder count
		driveport.write('d\r')
		denc = driveport.read(100)
		#denc = driveport.readline() #autonomous mode
		print(denc+"\n")

	elif key=="exit": #exit program
		driveport.write('f ') #send forward
		driveport.write("0")
		driveport.write('\r')
		steerport.write(b'\x80\x00\x00\x00')
		print("Exit")	
		driveport.close()
		steerport.close()
		exit()

	elif key=="dir": #directory
		print(" '0' to stop all motors,'a' for left,\n\r 'd' for right,\n\r 'l' to continue straight,\n\r 'w' to increase forward speed(+10),\n\r 's' to decrease forward speed(-10),\n\r 'B' to increase backwards speed(+5),\n\r 'b' to decrease backwards speed(-5),\n\r 'auto' to toggle simulated autonomy,\n\r 'u' to toggle E-Brake use,\n\r 'h on' to engage brake,\n\r 'h off' to disengage brake,\n\r 'senc' for steer encoder,\n\r 'denc' for drive encoder,\n\r 'exit' to exit program,\n\n\r Forward Speed = " + strspeed + "\n\r Backwards Speed = "  + bstrspeed + "\n\r")

	else:
		driveport.write('f ') #send forward
		driveport.write("0")
		driveport.write('\r')
		steerport.write('l\r')
		speed = 80
		print("YOU DIDN'T FOLLOW DIRECTIONS!")



	
driveport.write('f ') #send forward
driveport.write("0")
driveport.write('\r')
steerport.write(b'\x80\x00\x00\x00')
	
driveport.close()
steerport.close()
exit()
