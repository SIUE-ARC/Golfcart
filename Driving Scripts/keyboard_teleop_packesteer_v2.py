import serial
import time
from time import sleep # "sleep(seconds)"

driveport = serial.Serial("/dev/ttyUSB0", 19200)
steerport = serial.Serial("/dev/ttyUSB1", 9600)
serial.EIGHTBITS
serial.PARITY_NONE
serial.STOPBITS_ONE

steerport.write(b'\xaa') #for packet serial baud rate

key = raw_input("Enter Key a for left, d for right, w to increase speed, s to decrease speed, or enter an integer 87 - 254 to set speed: ")
 
speed = 80

while True:

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

	elif key== "b":
		strspeed = str(100)
		driveport.write('b ') #send forward
		driveport.write(strspeed) #send value to drive motor controller
		driveport.write('\r')
		print("New Speed is: "+ strspeed)

	elif key=="a": #turn left
		steerport.write(b'\x80\x00\x50\x50') #hex left >d'80 (128,0,80,80)

	elif key=="d": #turn right
		steerport.write(b'\x80\x01\x50\x51') #hex right <d'80 (128,1,80,81)

	elif key=="l": # go straight
		steerport.write(b'\x80\x00\x00\x00') #stops steer motor without stopping drive
		
	elif key=="0": #stop motors
		driveport.write('f ') #send forward
		driveport.write("0")
		driveport.write('\r')
		steerport.write(b'\x80\x00\x00\x00')
		speed = 80
		print("Motors stopped")

	elif key=="exit": #exit program
		driveport.write('f ') #send forward
		driveport.write("0")
		driveport.write('\r')
		steerport.write(b'\x80\x00\x00\x00')
		print("Exit")	
		driveport.close()
		steerport.close()
		exit()

	#elif ((varspeed >=86) & (varspeed <= 254)): #drive forward speed set
		#speed = int(key)
		#strspeed = str(speed)
		#driveport.write(strspeed) #send value to drive motor controller
		#driveport.write('\r')
		#print("New Speed is set to: "+ strspeed)

	else:
		driveport.write('f ') #send forward
		driveport.write("0")
		driveport.write('\r')
		steerport.write(b'\x80\x00\x00\x00')
		speed = 80
		print("YOU DIDN'T FOLLOW DIRECTIONS!")
		#driveport.close()
		#steerport.close()
		#exit()

	key = raw_input("Enter Key A for left, D for right, W for forward: ")
	#if (key != 'a' & key!='s' & key!='d' & key!='w' & key!='l' & key!=
	#varspeed = long(key)
	
	#if ((varspeed >= 80) & (varspeed <= 254)): #drive forward speed set
		#speed = int(key)
		#strspeed = str(speed)
		#driveport.write(strspeed) #send value to drive motor controller
		#driveport.write('\r')
		#print("New Speed is set to: "+ strspeed)


	
driveport.write('f ') #send forward
driveport.write("0")
driveport.write('\r')
steerport.write(b'\x80\x00\x00\x00')
	
driveport.close()
steerport.close()
exit()
