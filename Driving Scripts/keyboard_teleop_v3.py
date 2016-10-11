import serial
import time
from time import sleep # "sleep(seconds)"

driveport = serial.Serial("/dev/ttyUSB0", 115200)
steerport = serial.Serial("/dev/ttyUSB1", 38400)
serial.EIGHTBITS
serial.PARITY_NONE
serial.STOPBITS_ONE

#steerport.write(b'\xaa') #for packet serial baud rate

key = raw_input("Enter Key a for left, d for right, w to increase speed, s to decrease speed, or enter an integer 87 - 254 to set speed: ")
 
speed = 80

while True:


	if key== "w":
		speed = speed + 10
		strspeed = str(speed)
		driveport.write(strspeed) #send value to drive motor controller
		driveport.write('\r')
		print("New Speed is: "+ strspeed)
		
	elif key== "s":
		speed = speed - 10
		strspeed = str(speed)
		driveport.write(strspeed) #send value to drive motor controller
		driveport.write('\r')
		print("New Speed is: "+ strspeed)

	elif key=="a": #turn left
		steerport.write(b'\xc2') #hex forward >d'194

	elif key=="d": #turn right
		steerport.write(b'\x3c') #hex backwards <d'60

	elif key=="l": # go straight
		steerport.write(b'\x7f') #stops steer motor without stopping drive
		
	elif key=="0": #stop motors
		driveport.write("0")
		driveport.write('\r')
		steerport.write('\x7f')
		speed = 80
		print("Motors stopped")

	elif key=="exit": #exit program
		driveport.write("0".encode())
		steerport.write('\x7f')	
		print("Exit")	
		driveport.close()
		steerport.close()
		exit()

	elif (int(key) >=86) & (int(key) <= 254): #drive forward
		speed = int(key)
		strspeed = str(speed)
		driveport.write(strspeed) #send value to drive motor controller
		driveport.write('\r')
		print("New Speed is set to: "+ strspeed)

	else:
		driveport.write("0")
		driveport.write('\r')
		steerport.write('\x7f')
		speed = 80
		print("YOU DIDN'T FOLLOW DIRECTIONS!")


	key = raw_input("Enter Key A for left, D for right, W for forward: ")


	

driveport.write("0")
steerport.write('\x7f')
	
driveport.close()
steerport.close()
exit()
