__author__ = 'Ryan Owens'

import controllerClass
import serialCommunication
import time
import sys

drive_port = serialCommunication.BaseSerial("/dev/ttyUSB1", 19200, 0)
steer_port = serialCommunication.BaseSerial("/dev/ttyUSB0", 19200, 0)

current_speed = 80
stop_speed = 0
current_direction = "f "
min_move_speed = 85
counts_per_degree = 77.64232488822653

e_brake_flag = False
autonomous_flag = False
brake_flag = False

end_prog = False


def left_thumb_x(xValue):
    xValue *= 30
    count = convert_angle(xValue)
    print("LX" + str(xValue))
    print("Count: " + str(count))
    steer_port.send_command('t ', count, '\r')
    time.sleep(0.1)


def left_trigger(value):
    global current_direction
    global current_speed
    global min_move_speed
    value *= 254
    value = max(min_move_speed, value) // 1
    print("Left Trigger:" + str(value))
    drive_port.send_command('b ', value, '\r')
    current_direction = 'b '
    current_speed = value


def right_trigger(value):
    global current_direction
    global current_speed
    global min_move_speed
    value *= 254
    value = max(min_move_speed, value) // 1
    print("Right Trigger: " + str(value))
    drive_port.send_command('f ', value, '\r')
    current_direction = 'f '
    current_speed = value


def b_button(value):
    global end_prog
    print("B button pressed: ENDING")
    end_prog = True


# Sending a 'u' enables the brake to be on when E-Stop is physically pressed

def y_button(value):
    global e_brake_flag
    print("Flipping E-Brake")
    steer_port.send_command('u ', '\r')
    if e_brake_flag:
        e_brake_flag = False
    else:
        e_brake_flag = True


def a_button(value):
    global brake_flag
    if brake_flag:
        steer_port.send_command('h ', 'off', '\r')
        brake_flag = False
        print("Brake is off")
    else:
        brake_flag = True
        steer_port.send_command('h ', 'on', '\r')
        print("Brake is on")


def x_button(value):
    global current_direction
    global current_speed
    print("Halting")
    drive_port.send_command('f ', '0', '\r')
    steer_port.send_command('l ', '\r')
    current_speed = stop_speed
    current_direction = 'f '


def halt():
    drive_port.send_command('f ', '0', '\r')
    steer_port.send_command('t ', '0', '\r')
    drive_port.close_connection()
    steer_port.close_connection()


def convert_angle(angle):
    global counts_per_degree
    new_count = round(counts_per_degree * angle)
    return str(new_count)


def reset_steering(value):
    steer_port.send_command('r ', '\r')


if __name__ == '__main__':
    # for packet serial baud rate
    steer_port.send_command('i ', '\r')
    drive_port.send_command('i ', '\r')

    reset_steering('randomValue')

    # initial setup
    drive_port.send_command(current_direction, stop_speed, '\r')

    controller = controllerClass.Controller(controller_call_back=None, dead_zone=0.1, scale=1, invert_Y_axis=True,
                                            controller_is_xbox=True)
    xboxControls = controllerClass.XboxControls
    controller.setup_control_call_back(controller.controller_mapping.L_THUMB_X, left_thumb_x)
    # controller.setup_control_call_back(controller.controller_mapping.L_THUMB_Y, left_thumb_y)
    controller.setup_control_call_back(controller.controller_mapping.L_TRIGGER, left_trigger)
    controller.setup_control_call_back(controller.controller_mapping.R_TRIGGER, right_trigger)
    # controller.setup_control_call_back(controller.controller_mapping.R_THUMB_X, right_thumb_x)
    # controller.setup_control_call_back(controller.controller_mapping.R_THUMB_Y, right_thumb_y)
    controller.setup_control_call_back(controller.controller_mapping.B_BUTTON, b_button)
    controller.setup_control_call_back(controller.controller_mapping.A_BUTTON, a_button)
    controller.setup_control_call_back(controller.controller_mapping.RIGHT_BUMPER, reset_steering)

    try:
        controller.start()
        print("Controller startup")
        while not end_prog:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting")

    except:
        print("Unknown Error" + sys.exc_info()[0])
        raise

    finally:
        controller.stop()
        halt()