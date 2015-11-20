import serial
import time
from time import sleep # "sleep(seconds)"
from struct import *

port = serial.Serial("/dev/ttyUSB1", 9600)
serial.EIGHTBITS
serial.PARITY_NONE
serial.STOPBITS_ONE

port.write(b'\xaa') #170 for baud rate character 
address = 128 #constant for steer controller address

while 1:

	#get the speed to run and the time to run

	data = int(input("Enter Speed: "))
	wtime = int(input("Enter Time: "))

	# 0 for left, 1 for right -------------------------------------------------------------
	command = 0
	
	checksum = ((address + command + data) & 127)

	hexa = format(address,'02x')
	hexc = format(command,'02x')
	hexd = format(data,'02x')
	hexs = format(checksum,'02x')


	print address
	print command
	print data
	print checksum
	print ''
	print "Address is  ",hexa, "	",address
	print "Command is  ",hexc, "	",command
	print "Data is     ",hexd, "	",data
	print "Checksum is ",hexs, "	",checksum

	barray = [address,command,data,checksum]
	x = bytearray(barray)
	port.write(x)
	sleep(wtime)
	port.write(b'\x80\x00\x00\x00')# 130, 0, 64, 02(checksum) STOP


	# pause before going backwards--------------------------------------------
	key = raw_input("Press 'g' to go back")
	while key != 'g':
		key = raw_input("Press 'g' to go back")
		
							
		
	# go back right-----------------------------------------------------------
	command = 1
	
	checksum = ((address + command + data) & 127)

	hexa = format(address,'02x')
	hexc = format(command,'02x')
	hexd = format(data,'02x')
	hexs = format(checksum,'02x')


	print address
	print command
	print data
	print checksum
	print ''
	print "Address is  ",hexa, "	",address
	print "Command is  ",hexc, "	",command
	print "Data is     ",hexd, "	",data
	print "Checksum is ",hexs, "	",checksum

	barray = [address,command,data,checksum]
	x = bytearray(barray)
	port.write(x)
	sleep(wtime)
	port.write(b'\x80\x00\x00\x00')# 130, 0, 64, 02(checksum) STOP


port.write(b'\x80\x00\x00\x00')# 130, 0, 64, 02(checksum) STOP
sleep(1)
#port.write(b'\x82\x00\x00\x02')# 130, 0, 64, 02(checksum) STOP

port.close()
exit()
	

