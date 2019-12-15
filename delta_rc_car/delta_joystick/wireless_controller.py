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

import sys
import time
import traceback

from packet import Packet
from joystick import Joystick
from utils import *


class RCCarController():
    def __init__(self, joystick_port=0, serial_port='/dev/ttyACM0', publish_rate=10.0):
        self.joystick = Joystick(joystick_port)
        self.serial_port = serial_port
        self.publish_rate = publish_rate
        self.xbee = Packet()
        self.arm_car = False
        self.last_data = None
        self.arm_display_count = 0
        self.safety_check_done = False

    def setup(self):
        self.xbee.open(self.serial_port)

    def trigger_check(self):
        safe = False
        while not safe:
            data = self.joystick.parse_logitech()
            if data['left_trigger'] == -1.0 and data['right_trigger'] == -1.0:
                safe = True
            else:
                print('\nPlease press the left and right trigger (LT/RT) and release.')
                print('LT: %.1f, RT: %.1f (Both must be -1.0)' % (data['left_trigger'],
                    data['right_trigger']))
                input('>>> Press [ENTER] to continue.\n')
        self.safety_check_done = True

    def set_throttle_trigger(self, data):
        forward = mapfloat(data['right_trigger'], -1.0, 1.0, 0, 255)
        backward = mapfloat(data['left_trigger'], -1.0, 1.0, 0, 255)
        magnitude = constrain(int(max(forward, backward)), 0, 255)
        direction = 0 if forward >= backward else 1

        self.xbee.tx_throttle = magnitude
        self.xbee.tx_direction = direction

    def set_throttle_axis(self, data):
        magnitude = mapfloat(data['right_stick'][1], -1.0, 1.0, -255, 255)
        direction = 0 if magnitude > 0 else 1
        magnitude = constrain(abs(int(magnitude)), 0, 255)

        self.xbee.tx_throttle = magnitude
        self.xbee.tx_direction = direction

    def set_steering(self, data):
        steering = mapfloat(data['left_stick'][0], -1.0, 1.0, 0, 255)
        steering = constrain(int(steering), 0, 255)
        self.xbee.tx_steering = steering

    def set_arm(self, data):
        if data['face_buttons']['X'] and not self.last_data['face_buttons']['X']:
            if not self.arm_car: self.arm_car = True
            else: self.arm_car = False
        self.xbee.reset_packet()
        self.xbee.tx_arm = self.arm_car

    def run(self):
        # Parse joystick data.
        data = self.joystick.parse_logitech()

        # Safety checks.
        if not self.safety_check_done:
            self.trigger_check()
        if self.last_data is None:
            self.last_data = data
            return

        # Set data and send serial packet.
        self.set_arm(data)
        if self.arm_car:
            self.set_steering(data)
            # self.set_throttle_axis(data)
            self.set_throttle_trigger(data)
        self.xbee.send()

        self.display()
        self.last_data = data

    def loop(self):
        while True:
            try:
                time.sleep(1 / self.publish_rate)
                self.run()
            except KeyboardInterrupt:
                print('\n\n>>> USER STOPPED')
                break
            except Exception as error:
                print('\n\n>>> ERROR OCCURRED:', error)
                traceback.print_tb(error.__traceback__)
                break

        self.xbee.close()

    def display(self):
        # Arm text.
        if self.xbee.tx_arm:
            arm_text = '   ' if self.arm_display_count > 0.6 * self.publish_rate \
                             else '\033[1m\033[91mARM\033[0m'
        else: arm_text = 'OFF'
        self.arm_display_count = (self.arm_display_count + 1) % self.publish_rate

        # Steering text.
        if self.xbee.tx_steering == 127: str_text = '127'
        else: str_text = '\033[1m\033[93m%03d\033[0m' % self.xbee.tx_steering

        # Throttle text.
        if self.xbee.tx_throttle == 0: thr_text = 'NET 000'
        elif not self.xbee.tx_direction:
            thr_text = '\033[1m\033[92mFWD %03d\033[0m' % self.xbee.tx_throttle
        elif self.xbee.tx_direction:
            thr_text = '\033[1m\033[94mREV %03d\033[0m' % self.xbee.tx_throttle

        # Display all text.
        text = 'ARM: %s | THR: %s | STR: %s'
        text = text % (arm_text, thr_text, str_text)
        sys.stdout.write('\r' + text + ' ' * 5)
        sys.stdout.flush()


if __name__ == '__main__':
    rc = RCCarController(serial_port='/dev/ttyUSB0', publish_rate=50)
    rc.setup()
    rc.loop()
