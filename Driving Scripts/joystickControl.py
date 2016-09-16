import pygame
import serial
import time

driveport = serial.Serial("/dev/ttyUSB1", 19200, timeout = 0)
steerport = serial.Serial("/dev/ttyUSB0", 19200, timeout = 0)
serial.EIGHTBITS
serial.PARITY_NONE
serial.STOPBITS_ONE

steerport.write('i\r') #for packet serial baud rate
time.sleep(2)
steerport.write('r\r') #for packet serial baud rate

driveport.write('a\r')


pygame.init();

print "Joystics: ", pygame.joystick.get_count()

joystick = pygame.joystick.Joystick(0)
joystick.init()

clock = pygame.time.Clock()


def setThrottle(speed):	
	control = int(round(speed * 80))
	print "Setting throttle to " + str(80 + control)
	if control > 10:
		driveport.write('f '+str(80 + abs(control)) + '\r') #forward
	elif control < -10:
		driveport.write('b '+str(80 + abs(control)) + '\r') #backwards
	else:
		driveport.write('f 0\r')


	# Set drive psoc speed over serial

def turnTo(target):
	control = int(round(target * 600));
	print "Setting turn to " + str(control)
	if control > 10:
		steerport.write('t '+str(control) + '\r') #forward
	elif control < -10:
		steerport.write('t '+str(control) + '\r') #backwards
	else:
		steerport.write('T 0\r')

	# Set turn target over serial

def setBrake(brake):
	control = int(round(brake * 800))
	print "Setting brake to: " + str(control);
	steerport.write('h ' + str(control) + '\r');

while 1:
	for event in pygame.event.get():
		pass

	throttle = -joystick.get_axis(1);
	wheel = joystick.get_axis(0);
	brake = (-joystick.get_axis(2) + 1.0) / 2.0;
	
	setThrottle(throttle);
	setBrake(brake);
	turnTo(wheel);

	clock.tick(10)

pygame.quit ()

