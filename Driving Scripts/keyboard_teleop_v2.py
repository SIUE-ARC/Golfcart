import serial
import time
from time import sleep # "sleep(seconds)"

driveport = serial.Serial("/dev/ttyUSB0", 115200)
steerport = serial.Serial("/dev/ttyUSB1", 38400)
serial.EIGHTBITS
serial.PARITY_NONE
serial.STOPBITS_ONE

#steerport.write(b'\xaa') #for packet serial baud rate

key = raw_input("Enter Key A for left, D for right, W for forward: ")
while True:
	if key == "w": #drive forward
		speed = raw_input("Drive Motor Speed (87 - 254): ")
		driveport.write(speed) #send value to drive motor controller
		driveport.write('\r')
		print("New Speed is: "+ speed)
		
	elif key=="a": #turn left
		steerport.write(b'\xc2') #hex forward >d'194

	elif key=="d": #turn right
		steerport.write(b'\x3c') #hex backwards <d'60
		
	elif key=="0": #stop motors
		driveport.write("0")
		driveport.write('\r')
		steerport.write('\x7f')
	elif key=="exit": #exit program
		driveport.write("0".encode())
		steerport.write('\x7f')
	
		driveport.close()
		steerport.close()
		exit()

	key = raw_input("Enter Key A for left, D for right, W for forward: ")

	

driveport.write("0")
steerport.write('\x7f')
	
driveport.close()
steerport.close()
exit()
