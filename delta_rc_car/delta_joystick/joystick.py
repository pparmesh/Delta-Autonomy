#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.0
Date    : Oct 31, 2019
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

import pygame


class Joystick():
    def __init__(self, joystick_port=0):        
        # Setup joystick.
        pygame.init()
        self.joystick_port = joystick_port
        self.joystick = pygame.joystick.Joystick(self.joystick_port)
        self.joystick.init()
        print('>>> Initialized Joystick: ' + self.joystick.get_name())

    def read(self):
        data = [0] * self.joystick.get_numaxes()
        data += [0] * self.joystick.get_numbuttons()
        data += [0]
        it = 0 # iterator
        pygame.event.pump()

        # Read input from the two joysticks       
        for i in range(self.joystick.get_numaxes()):
            data[it] = self.joystick.get_axis(i)
            it += 1

        # Read input from buttons
        for i in range(self.joystick.get_numbuttons()):
            data[it] = self.joystick.get_button(i)
            it += 1

        data[it] = self.joystick.get_hat(0)  
        return data

    def parse_logitech(self):
        data = self.read()

        parsed_dict = {}
        # Axis data.
        parsed_dict['left_stick'] = (data[0], -data[1])
        parsed_dict['right_stick'] = (data[3], -data[4])
        parsed_dict['left_trigger'] = data[2]
        parsed_dict['right_trigger'] = data[5]
        parsed_dict['left_bumper'] = data[10]
        parsed_dict['right_bumper'] = data[11]

        # Buttons data.
        parsed_dict['face_buttons'] = {
            'A': bool(data[6]),
            'B': bool(data[7]),
            'X': bool(data[8]),
            'Y': bool(data[9])}
        parsed_dict['back'] = bool(data[12])
        parsed_dict['start'] = bool(data[13])
        parsed_dict['left_stick_button'] = bool(data[15])
        parsed_dict['right_stick_button'] = bool(data[16])

        # Direction pad data.
        parsed_dict['dpad'] = data[17]

        return parsed_dict
