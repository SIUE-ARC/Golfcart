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
 
speed = 100
strspeed = str(speed)


if key== "w":
		
		 # go straight
		steerport.write(b'\x80\x00\x00\x00') #stops steer motor without stopping drive


		driveport.write('f ') #send forward
		driveport.write(strspeed) #send value to drive motor controller
		driveport.write('\r')
		print("Its going!")
		sleep(15)

		
 		#stop motors
		driveport.write('f ') #send forward
		driveport.write("0")
		driveport.write('\r')
		steerport.write(b'\x80\x00\x00\x00')
		print("Motors stopped")

		 # go straight
		steerport.write(b'\x80\x00\x00\x00') #stops steer motor without stopping drive
		sleep(3)


		speed = 110
		strspeed = str(speed)
		driveport.write('b ') #send forward
		driveport.write(strspeed) #send value to drive motor controller
		driveport.write('\r')
		print("Its going back!")
		sleep(16)

				
 		#stop motors
		driveport.write('f ') #send forward
		driveport.write("0")
		driveport.write('\r')
		steerport.write(b'\x80\x00\x00\x00')
		print("Motors stopped")
		print("Done")


else:
		driveport.write('f ') #send forward
		driveport.write("0")
		driveport.write('\r')
		steerport.write(b'\x80\x00\x00\x00')
		print("YOU DIDN'T FOLLOW DIRECTIONS!")
		#driveport.close()
		#steerport.close()
		#exit()


	
driveport.write('f ') #send forward
driveport.write("0")
driveport.write('\r')
steerport.write(b'\x80\x00\x00\x00')
	
driveport.close()
steerport.close()
exit()
