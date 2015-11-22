__author__ = 'Ryan Owens'
import gi
try:
    gi.require_version("Gtk", "3.0")
except ValueError:
    print("Requires gtk3 development files to be installed.")
    exit(1)
except AttributeError:
    print("pygobject version too old.")
    exit(2)

from gi.repository import Gtk

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


def left_thumb_x(x_value):
    x_value *= 30
    count = convert_angle(x_value)
    win.add_to("Angle: " + str(x_value), "Controller")
    steer_port.send_command('t ', count, '\r')
    win.add_to(steer_port.get_response(), "Steer Port")
    time.sleep(0.1)


def left_trigger(value):
    global current_direction
    global current_speed
    global min_move_speed
    value *= 254
    value = max(min_move_speed, value) // 1
    win.add_to("Forward Speed: " + str(value), "Controller")
    drive_port.send_command('b ', value, '\r')
    win.add_to(drive_port.get_response(), "Drive Port")
    current_direction = 'b '
    current_speed = value


def right_trigger(value):
    global current_direction
    global current_speed
    global min_move_speed
    value *= 254
    value = max(min_move_speed, value) // 1
    win.add_to("Backward Speed: " + str(value), "Controller")
    drive_port.send_command('f ', value, '\r')
    win.add_to(drive_port.get_response(), "Drive Port")
    current_direction = 'f '
    current_speed = value


def b_button(value):
    win.add_to("B button pressed. Ending Program", "Controller")
    halt()
    Gtk.main_quit()


# Sending a 'u' enables the brake to be on when E-Stop is physically pressed

def y_button(value):
    global e_brake_flag
    win.add_to("Y Button Pressed: Turning on E-Brake", "Controller")
    steer_port.send_command('u ', '\r')
    win.add_to(steer_port.get_response(), "Steer Port")
    if e_brake_flag:
        e_brake_flag = False
    else:
        e_brake_flag = True


def a_button(value):
    global brake_flag
    if brake_flag:
        steer_port.send_command('h ', 'off', '\r')
        brake_flag = False
        win.add_to("A Button Pressed: Brake is off", "Controller")
    else:
        brake_flag = True
        steer_port.send_command('h ', 'on', '\r')
        win.add_to("A Button Pressed: Brake is on", "Controller")
    win.add_to(steer_port.get_response(), "Steer Port")


def x_button(value):
    global current_direction
    global current_speed
    win.add_to("X Button Pressed: Halting", "Controller")
    drive_port.send_command('f ', '0', '\r')
    win.add_to(drive_port.get_response(), "Drive Port")
    steer_port.send_command('l ', '\r')
    win.add_to(steer_port.get_response(), "Steer Port")
    current_speed = stop_speed
    current_direction = 'f '


def halt():
    drive_port.send_command('f ', '0', '\r')
    win.add_to(drive_port.get_response(), "Drive Port")
    steer_port.send_command('t ', '0', '\r')
    win.add_to(steer_port.get_response(), "Steer Port")
    drive_port.close_connection()
    steer_port.close_connection()


def convert_angle(angle):
    global counts_per_degree
    new_count = round(counts_per_degree * angle)
    return str(new_count)


def reset_steering(value):
    steer_port.send_command('r ', '\r')
    if value is not ' ':
        win.add_to("Sending Steer Reset", "Controller")
    else:
        win.add_to("Sending Steer Reset", "Program Startup")
    win.add_to(steer_port.get_response(), "Steer Port")


def on_controller_disconnect():
    win.add_to("Controller has been disconnected: Waiting for Reconnect", "Controller Error")
    drive_port.send_command('f ', '0', '\r')
    win.add_to(drive_port.get_response(), "Drive Port")
    steer_port.send_command('t ', '0', '\r')
    win.add_to(steer_port.get_response(), "Steer Port")


class ResponseWindow(Gtk.Window):

    def __init__(self):

        Gtk.Window.__init__(self, title='I was bored')
        self.set_default_size(600, 300)
        self.connect('delete-event', Gtk.main_quit)

        self.__box = Gtk.VBox(spacing=10)

        self.__store = Gtk.ListStore(str, str)
        self.__tree_view = Gtk.TreeView(model=self.__store)
        self.__tree_view.connect('size-allocate', self.tree_view_changed)

        self.__renderer_1 = Gtk.CellRendererText()
        self.__column_1 = Gtk.TreeViewColumn('Data', self.__renderer_1, text=0)
        self.__tree_view.append_column(self.__column_1)

        self.__renderer_2 = Gtk.CellRendererText(xalign=1)
        self.__column_2 = Gtk.TreeViewColumn('To/From', self.__renderer_2, text=1)
        self.__tree_view.append_column(self.__column_2)
        self.__scrolled_window = Gtk.ScrolledWindow()
        self.__scrolled_window.set_policy(
            Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        self.__scrolled_window.add(self.__tree_view)
        self.__scrolled_window.set_min_content_height(200)

        self.__column_1.set_expand(True)
        self.__column_2.set_expand(False)

        self.__entry = Gtk.Entry()
        self.__entry.connect("activate", self.enter_callback, self.__entry)

        self.__drive_port_button = Gtk.ToggleButton("Drive Port")
        self.__drive_port_button.set_active(True)
        self.__drive_port_button.connect("toggled", self.on_drive_button_toggled, "1")

        self.__steer_port_button = Gtk.ToggleButton("Steer Port")
        self.__steer_port_button.connect("toggled", self.on_steer_button_toggled, "2")

        self.__buttonBox = Gtk.Box(spacing=10)
        self.__buttonBox.pack_start(self.__drive_port_button, True, True, 0)
        self.__buttonBox.pack_start(self.__steer_port_button, True, True, 0)

        self.__box.pack_start(self.__scrolled_window, True, True, 0)
        self.__box.pack_start(self.__buttonBox, False, False, 0)
        self.__box.pack_start(self.__entry, False, False, 10)

        self.add(self.__box)
        self.show_all()

    def add_to(self, value1, value2):
        self.__store.append([str(value1), str(value2)])

    def enter_callback(self, widget, entry):
        entry_text = entry.get_text()
        if self.__drive_port_button.get_active():
            self.add_to(entry_text, "Keyboard->Drive Port")
            drive_port.send_command(entry_text)
            win.add_to(drive_port.get_response(), "Drive Port")
        else:
            self.add_to(entry_text, "Keyboard->Steer Port")
            steer_port.send_command(entry_text)
            win.add_to(steer_port.get_response(), "Steer Port")
        entry.set_text("")

    def tree_view_changed(self, widget, event, data=None):
        v_adjustment = self.__scrolled_window.get_vadjustment()
        v_adjustment.set_value(v_adjustment.get_upper() - v_adjustment.get_page_size())

    def on_drive_button_toggled(self, button, name):
        toggle_val = self.__drive_port_button.get_active()
        self.__steer_port_button.set_active(not toggle_val)

    def on_steer_button_toggled(self, button, name):
        toggle_val = self.__steer_port_button.get_active()
        self.__drive_port_button.set_active(not toggle_val)


if __name__ == '__main__':
    win = ResponseWindow()
    # for packet serial baud rate
    steer_port.send_command('i ', '\r')
    win.add_to("Sending i to Steer Port", "Program Startup")
    drive_port.send_command('i ', '\r')
    win.add_to("Sending i to Drive Port", "Program Startup")

    reset_steering(' ')

    # initial setup
    drive_port.send_command(current_direction, stop_speed, '\r')

    controller = controllerClass.Controller(controller_call_back=None, dead_zone=0.1, scale=1, invert_Y_axis=True,
                                            controller_is_xbox=True, controller_disconnect=on_controller_disconnect)
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
        Gtk.main()

    except KeyboardInterrupt:
        print("Exiting")

    except:
        print("Unknown Error" + sys.exc_info()[0])
        raise

    finally:
        controller.stop()
        halt()
