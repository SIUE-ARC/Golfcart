import pygame
import serial

driveport = serial.Serial("/dev/ttyUSB0", 19200, timeout = 0)
#steerport = serial.Serial("/dev/ttyUSB1", 19200, timeout = 0)
serial.EIGHTBITS
serial.PARITY_NONE
serial.STOPBITS_ONE

#steerport.write('i\r') #for packet serial baud rate
sleep(2)
#steerport.write('r\r') #reset
driveport.write('a\r')


pygame.init();

print "Joystics: ", pygame.joystick.get_count()

joystick = pygame.joystick.Joystick(0)
joystick.init()

clock = pygame.time.Clock()


def setThrottle(speed):	
	control = int(round(speed * 255))
	print "Setting throttle to " + str(control)
	if control > 10:
		driveport.write('f '+abs(control) + '\r') #forward
	elif control < -10:
		driveport.write('b '+abs(control) + '\r') #backwards
	else:
		driveport.write('f 0\r')


	# Set drive psoc speed over serial

def turnTo(target):
	control = int(round(target * 600));
	print "Setting turn to " + str(control)

	# Set turn target over serial

while 1:
	for event in pygame.event.get():
		pass

	throttle = -joystick.get_axis(1);
	wheel = joystick.get_axis(0);
	
	setThrottle(throttle);
	#turnTo(wheel);

	clock.tick(40)

pygame.quit ()

