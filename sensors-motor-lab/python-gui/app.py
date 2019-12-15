#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Python GUI for Sensors and Motors Lab
'''

__author__ = "Heethesh Vhavle"
__version__ = "1.0.0"
__email__ = "heethesh@cmu.edu"

# Built-in modules
import os
import glob
import time

# External modules
import numpy as np
from tkinter import *
from tkinter import ttk
from tkinter.ttk import Separator, Progressbar, Notebook
from PIL import ImageTk, Image
import matplotlib

matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Local modules
from utils import *
from packet import Packet

# Globals
PATH = os.path.dirname(os.path.abspath(__file__)) + '/'
HOME_PATH = os.path.expanduser('~') + '/'

app = None
GUI_CLOSED = False

# Arduino packet object
arduino = Packet()
last_time = time.time()

STATES = [
    'Reserved',
    'DC Motor Position (GUI)',
    'DC Motor Velocity (GUI)',
    'DC Motor Position (Sensor)',
    'DC Motor Velocity (Sensor)',
    'Stepper Motor (GUI)',
    'Stepper Motor (Sensor)',
    'Servo Motor (GUI)',
    'Servo Motor (Sensor)',
]

'''
Function wrapper to send data packet to arduino
'''
def arduino_send():
    global last_time
    if (time.time() - last_time) > 0.005 and arduino.is_open:
        last_time = time.time()
        arduino.send()


'''
Class to handle the sensor panel using a progress bar
'''
class SensorPanel:
    def __init__(self, name, min_val, max_val, parent, row, column, padx):
        # Sensor properties
        self.name = name
        self.min_val = min_val
        self.max_val = max_val

        # Append to Tk parent panel
        self.panel = Frame(parent, width=170, height=10, pady=3)
        if name:
            self.name_label = Label(self.panel, text=name)
            self.name_label.grid(row=0, column=0, columnspan=2, pady=5)
        self.pb = Progressbar(
            self.panel, orient=HORIZONTAL, mode="determinate", length=140
        )
        self.pb.grid(row=1, column=0, sticky=(W, E))
        self.value_label = Entry(self.panel, width=7, justify=RIGHT)
        self.value_label.grid(row=1, column=1, padx=5, sticky=(W, E))
        self.panel.grid(row=row, column=column, padx=padx)
        self.set_sensor_value(0)

    def set_sensor_value(self, value):
        self.pb['value'] = map_value(value, self.min_val, self.max_val)
        self.value_label.delete(0, END)
        self.value_label.insert(0, '%.2f' % value)


'''
Class to handle the slider panel using a slider
'''
class SliderPanel:
    def __init__(self, name, min_val, max_val, parent, row, column, padx, func):
        # Sensor properties
        self.name = name
        self.min_val = min_val
        self.max_val = max_val

        # Append to Tk parent panel
        self.panel = Frame(parent, width=170, pady=3)
        self.name_label = Label(self.panel, text=name)
        self.name_label.grid(row=0, column=0, padx=6, sticky=(S))
        self.slider = Scale(
            self.panel,
            from_=min_val,
            to=max_val,
            orient=HORIZONTAL,
            length=140,
            command=func,
        )
        self.slider.grid(row=0, column=1, sticky=(W, E))
        self.panel.grid(row=row, column=column, padx=padx)


'''
Class to handle the titles used for the panels
'''
class SectionTitle:
    def __init__(self, title, parent, row, column, width, top=10, bottom=10):
        Separator(parent, orient=HORIZONTAL).grid(
            row=row, column=column, pady=(top, bottom), sticky=(W, E)
        )
        self.panel = Frame(parent, width=width, height=10)
        self.panel.grid(row=row, column=column)
        self.label = Label(self.panel, text=" %s " % title)
        self.label.grid(row=row, column=column, pady=(top, bottom))


'''
Class to handle the eight different state panels
'''
class StatePanel:
    def __init__(self, state, parent, row, column, width, padx):
        self.state_index = STATES.index(state)
        self.panel = Frame(parent, width=width, pady=3)
        self.raisedPanel = Frame(self.panel, width=width, pady=3, 
            bd=1, relief=RAISED)
        SectionTitle(state, self.panel, row=0, column=0, width=170)

        # DC motor position GUI
        if self.state_index == 1:
            self.slider1 = SliderPanel(
                'Angle',
                -360,
                360,
                self.raisedPanel,
                row=0,
                column=0,
                padx=9,
                func=self.control_dcmotor_pos,
            )
            self.sensor1 = SensorPanel(
                'Encoder Position (Ticks)',
                -10000,
                10000,
                self.raisedPanel,
                row=1,
                column=0,
                padx=9,
            )

        # DC motor velocity GUI
        elif self.state_index == 2:
            self.slider1 = SliderPanel(
                'Velocity',
                0,
                110,
                self.raisedPanel,
                row=0,
                column=0,
                padx=9,
                func=self.control_dcmotor_vel,
            )
            self.sensor1 = SensorPanel(
                'Encoder Velocity (RPM)',
                -110,
                110,
                self.raisedPanel,
                row=1,
                column=0,
                padx=9,
            )

        # DC motor position sensor
        elif self.state_index == 3:
            self.sensor1 = SensorPanel(
                'Encoder Position (Ticks)',
                -1000000,
                1000000,
                self.raisedPanel,
                row=0,
                column=0,
                padx=9,
            )
            self.sensor2 = SensorPanel(
                'Temperature Sensor (C)',
                0,
                200,
                self.raisedPanel,
                row=1,
                column=0,
                padx=9,
            )

        # DC motor velocity sensor
        elif self.state_index == 4:
            self.sensor1 = SensorPanel(
                'Encoder Velocity (RPM)',
                -110,
                110,
                self.raisedPanel,
                row=0,
                column=0,
                padx=9,
            )
            self.sensor2 = SensorPanel(
                'Ultrasonic Sensor (inches)',
                6,
                24,
                self.raisedPanel,
                row=1,
                column=0,
                padx=9,
            )

        # Stepper motor GUI control
        elif self.state_index == 5:
            self.slider1 = SliderPanel(
                'Angle',
                0,
                360,
                self.raisedPanel,
                row=0,
                column=0,
                padx=9,
                func=self.control_stepper_pos,
            )
            self.slider2 = SliderPanel(
                'Direction',
                0,
                1,
                self.raisedPanel,
                row=1,
                column=0,
                padx=9,
                func=self.control_stepper_dir,
            )
            self.sensor1 = SensorPanel(
                'Slot Encoder', 0, 1, self.raisedPanel, row=2, column=0, padx=9
            )
            self.button = Button(
                self.raisedPanel,
                text="Set Angle",
                command=self.set_stepper_flag,
                pady=4,
                width=24,
            )
            self.button.grid(row=3, column=0, pady=3)

        # Stepper motor sensor control
        elif self.state_index == 6:
            self.sensor1 = SensorPanel(
                'Slot Encoder', 0, 1, self.raisedPanel, row=1, column=0, padx=9
            )

        # Servo motor GUI control
        elif self.state_index == 7:
            self.slider1 = SliderPanel(
                'Angle',
                0,
                90,
                self.raisedPanel,
                row=0,
                column=0,
                padx=9,
                func=self.control_servo,
            )
            self.sensor1 = SensorPanel(
                'Flex Sensor', 0, 1024, self.raisedPanel, row=1, column=0, padx=9
            )

        # Servo motor sensor control
        elif self.state_index == 8:
            self.sensor1 = SensorPanel(
                'Flex Sensor', 0, 1024, self.raisedPanel, row=1, column=0, padx=9
            )

        self.raisedPanel.grid(row=1, column=0, padx=padx)
        self.panel.grid(row=row, column=column, padx=padx)
        self.configure_state(self.raisedPanel, state=DISABLED)

    '''Callback functions for all states'''

    def control_servo(self, value):
        arduino.rx_servo_angle = int(value)
        arduino_send()

    def control_stepper_pos(self, value):
        arduino.rx_stepper_value = int(value)

    def control_stepper_dir(self, value):
        arduino.rx_stepper_dir = int(value)

    def set_stepper_flag(self):
        arduino.rx_stepper_flag = 1
        arduino_send()

    def control_dcmotor_pos(self, value):
        arduino.rx_motor_angle = int(value)
        arduino_send()

    def control_dcmotor_vel(self, value):
        arduino.rx_motor_velocity = int(value)
        arduino_send()

    def configure_state(self, frame, state):
        for child in frame.winfo_children():
            if type(child) == Frame:
                self.configure_state(child, state)
            elif type(child) == Progressbar:
                continue
            else:
                child.configure(state=state)


'''
Main class to handle the GUI
'''
class GUI(object):
    def __init__(self, master):
        # Main Window
        self.width = 820
        self.height = 645
        master.title("Sensors and Motors Lab  |  Delta Autonomy")
        master.geometry('%dx%d' % (self.width, self.height))
        master.resizable(width=False, height=False)

        self.ICON_PATH = PATH + 'images/da_logo_resize.gif'
        self.imgicon = PhotoImage(file=self.ICON_PATH)
        master.tk.call('wm', 'iconphoto', master._w, self.imgicon)

        ###################################################################

        # Master Panel
        self.mpanel = Frame(
            master, width=self.width, height=self.height, padx=5, pady=4
        )
        self.mpanel.pack()

        ###################################################################

        # Left Panel
        self.lpanel = Frame(self.mpanel, width=170, height=self.height, pady=3)
        self.lpanel.grid(row=0, column=0, rowspan=4, padx=10)

        ###################################################################

        # Info Panel
        self.raisedFrame = Frame(self.lpanel, bd=3, relief=GROOVE)
        self.raisedFrame.grid(row=0, column=0)

        self.logo = ImageTk.PhotoImage(Image.open(PATH + 'images/da_logo_resize.gif'))
        self.logolabel = Label(self.raisedFrame, image=self.logo, width=205)
        self.logolabel.grid(row=0, column=0)

        self.infolabel3 = Label(self.raisedFrame, text=" ", font='"Consolas" 2')
        self.infolabel3.grid(row=1, column=0)

        self.infolabel = Label(
            self.raisedFrame, text="Delta Autonomy", font='"Consolas" 12 bold'
        )
        self.infolabel.grid(row=2, column=0)

        self.infolabel2 = Label(
            self.raisedFrame,
            text="Sensors and Motors Lab GUI\nVersion %s" % __version__,
            font='"Consolas" 10',
        )
        self.infolabel2.grid(row=3, column=0)

        self.infolabel5 = Label(self.raisedFrame, text=" ", font='"Consolas" 4')
        self.infolabel5.grid(row=4, column=0)

        self.infolabel6 = Label(
            self.lpanel, text="\nInstructions", font='"Consolas" 11 bold'
        )
        self.infolabel6.grid(row=3, column=0)

        self.infotext = "Select the COM port and open it. Select any one of the eight states and click start demo to enable the corresponding state panel. You can now visualize the sensors data and control the actuators."
        self.infolabel4 = Label(
            self.lpanel, text=self.infotext, wraplength=205, font='"Consolas" 9'
        )
        self.infolabel4.grid(row=4, column=0)

        ###################################################################

        # COM Port Panel
        SectionTitle('Select COM Port', self.lpanel, 5, 0, 170)

        self.comport = StringVar(master)
        self.comports = self.get_com_ports()
        if not self.comports:
            self.comports = ['No Devices Available']
        self.ddcom = OptionMenu(
            self.lpanel, self.comport, *self.comports, command=self.comport_select
        )
        self.ddcom.config(width=22)
        self.ddcom.grid(row=6, column=0)

        self.b1 = Button(
            self.lpanel, text="Open Port", command=self.b1_clicked,
            pady=4, width=24
        )
        self.b1.grid(row=7, column=0)
        self.b1.configure(state=DISABLED)

        ###################################################################

        # State Select Panel
        SectionTitle('Select State', self.lpanel, 8, 0, 170)

        self.state = StringVar(master)
        self.states = STATES[1:]
        self.ddstate = OptionMenu(
            self.lpanel, self.state, *self.states, command=self.state_select
        )
        self.ddstate.config(width=22)
        self.ddstate.grid(row=9, column=0)

        self.b2 = Button(
            self.lpanel, text="Start Demo", command=self.b2_clicked, 
            pady=4, width=24
        )
        self.b2.grid(row=10, column=0)
        self.b2.configure(state=DISABLED)

        ###################################################################

        # Separator
        Separator(self.mpanel, orient=VERTICAL).grid(
            row=0, column=1, rowspan=20, sticky=(N, S), padx=6
        )

        # Right Panel
        self.rpanel = Frame(self.mpanel, width=170, height=self.height, pady=3)
        self.rpanel.grid(row=0, column=2, rowspan=4)

        # State Panels
        self.state1_panel = StatePanel(
            STATES[1], self.rpanel, row=0, column=0, width=170, padx=9
        )
        self.state3_panel = StatePanel(
            STATES[3], self.rpanel, row=1, column=0, width=170, padx=9
        )
        self.state2_panel = StatePanel(
            STATES[2], self.rpanel, row=2, column=0, width=170, padx=9
        )
        self.state4_panel = StatePanel(
            STATES[4], self.rpanel, row=3, column=0, width=170, padx=9
        )

        ###################################################################

        # Separator
        Separator(self.mpanel, orient=VERTICAL).grid(
            row=0, column=3, rowspan=20, sticky=(N, S), padx=6
        )

        # Right Panel
        self.r2panel = Frame(self.mpanel, width=170, height=self.height, pady=3)
        self.r2panel.grid(row=0, column=4, rowspan=4)

        # State Panels
        self.state5_panel = StatePanel(
            STATES[5], self.r2panel, row=0, column=0, width=170, padx=9
        )
        self.state6_panel = StatePanel(
            STATES[6], self.r2panel, row=1, column=0, width=170, padx=9
        )
        self.state7_panel = StatePanel(
            STATES[7], self.r2panel, row=2, column=0, width=170, padx=9
        )
        self.state8_panel = StatePanel(
            STATES[8], self.r2panel, row=3, column=0, width=170, padx=9
        )

        ###################################################################

        # Keep track all state panel objects
        self.current_state = STATES[0]
        self.state_panels = [
            self.state1_panel,
            self.state2_panel,
            self.state3_panel,
            self.state4_panel,
            self.state5_panel,
            self.state6_panel,
            self.state7_panel,
            self.state8_panel,
        ]

        ###################################################################

        # Separator
        Separator(self.mpanel, orient=VERTICAL).grid(
            row=0, column=3, rowspan=20, sticky=(N, S), padx=6
        )

        ###################################################################

    '''Callback functions'''

    def get_com_ports(self):
        # Linux
        return glob.glob('/dev/tty[AU]*')

    def comport_select(self, port):
        print('Port changed:', port)
        self.b1.configure(state=NORMAL)

    def state_select(self, state):
        print('State changed:', STATES.index(state), state)
        self.b2.configure(state=NORMAL)
        self.current_state = state

    def b1_clicked(self):
        print('B1 Clicked')
        if self.b1['text'] == 'Close Port':
            self.b1['text'] = 'Open Port'
            self.ddcom.configure(state=NORMAL)
            arduino.close()

        elif self.b1['text'] == 'Open Port':
            self.b1['text'] = 'Close Port'
            self.ddcom.configure(state=DISABLED)
            print('Opening Port:', str(self.comport.get()))
            arduino.start(str(self.comport.get()))

    def b2_clicked(self):
        print('B2 Clicked')
        if self.b2['text'] == 'Stop Demo':
            self.b2['text'] = 'Start Demo'
            self.ddstate.configure(state=NORMAL)
            state_index = STATES.index(self.current_state)
            if state_index:
                self.state_panels[state_index - 1].configure_state(
                    self.state_panels[state_index - 1].raisedPanel, state=DISABLED
                )
            arduino.rx_global_switch = 0
            arduino.rx_state = state_index
            arduino_send()

        elif self.b2['text'] == 'Start Demo':
            self.b2['text'] = 'Stop Demo'
            self.ddstate.configure(state=DISABLED)
            state_index = STATES.index(self.current_state)
            if state_index:
                self.state_panels[state_index - 1].configure_state(
                    self.state_panels[state_index - 1].raisedPanel, state=NORMAL
                )
            arduino.rx_global_switch = 1
            arduino.rx_state = state_index
            arduino_send()

    '''
    Function to update data on the GUI from serial packets
    '''
    def update_data(self):
        self.state1_panel.sensor1.set_sensor_value(arduino.tx_encoder['encoder_count'])
        self.state2_panel.sensor1.set_sensor_value(
            arduino.tx_encoder['encoder_velocity']
        )
        self.state3_panel.sensor1.set_sensor_value(arduino.tx_encoder['encoder_count'])
        self.state3_panel.sensor2.set_sensor_value(arduino.tx_temperature)
        self.state4_panel.sensor1.set_sensor_value(
            arduino.tx_encoder['encoder_velocity']
        )
        self.state4_panel.sensor2.set_sensor_value(arduino.tx_ultrasonic_distance)
        self.state5_panel.sensor1.set_sensor_value(int(bool(arduino.tx_slot_encoder)))
        self.state6_panel.sensor1.set_sensor_value(int(bool(arduino.tx_slot_encoder)))
        self.state7_panel.sensor1.set_sensor_value(arduino.tx_flex_sensor)
        self.state8_panel.sensor1.set_sensor_value(arduino.tx_flex_sensor)


'''
Threaded function to keep listening to packets over serial
'''
def packet_listener():
    global app
    time.sleep(2)

    # Keep running thread till GUI is open
    while not GUI_CLOSED:
        if arduino.is_open and arduino.recieve():
            app.update_data()
        else:
            time.sleep(0.1)


if __name__ == '__main__':
    # Start thread
    packet_listener_t = StoppableThread(target=packet_listener)
    packet_listener_t.start()

    # Create GUI and start GUI thread
    root = Tk()
    root.wm_attributes('-type', 'splash')
    app = GUI(root)
    root.mainloop()

    # Cleanup
    GUI_CLOSED = True
    packet_listener_t.stop()
    if arduino.is_open: arduino.close()
