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

import time
import serial
import struct
import traceback


class Packet:
    def __init__(self):
        self.is_open = False
        self.reset_packet()

    def reset_packet(self):
        # TX data
        self.tx_arm = 0
        self.tx_steering = 127
        self.tx_throttle = 0
        self.tx_direction = 0

        # RX data
        self.rx_state = None

    def open(self, com_port, baud=9600, timeout=0):
        # Configure serial port
        self.ser = serial.Serial()
        self.ser.port = com_port
        self.ser.baudrate = baud
        self.ser.timeout = timeout

        # Time to wait until the board becomes operational
        wakeup = 2
        try:
            self.ser.open()
            print("\n>>> Opening Serial Port: " + self.ser.port)
            for i in range(1, wakeup): time.sleep(1)
        except Exception as error:
            traceback.print_tb(error.__traceback__)
            self.ser.close()
            self.is_open = False

        # Clear buffer
        self.ser.flushInput()
        self.ser.flushOutput()
        self.is_open = True

    def close(self):
        try:
            self.ser.close()
            print("\n>>> Closing Serial Port: " + self.ser.port)
        except AttributeError as e:
            print(e)
        self.is_open = False

    def generate_frame(self, data, data_format, mode):
        checksum = 0
        frame = ''
        direc = {'tx': '<', 'rx': '>'}

        # Pack data into bytes
        header = struct.pack('cc', '$'.encode('utf-8'), direc[mode].encode('utf-8'))
        payload = struct.pack(data_format, *data)
        data_length = struct.pack('B', len(payload))

        # Calculate checksum
        for byte in payload:
            checksum ^= byte
        checksum = struct.pack('B', checksum)

        # Complete frame
        frame = header + data_length + payload + checksum
        return frame

    def send_packet(self, data, data_format, mode='tx'):
        # Make frame
        tx_frame = self.generate_frame(data, data_format, mode)

        # Send data
        try:
            self.ser.write(tx_frame)
        except Exception as error:
            print(error)
            traceback.print_tb(error.__traceback__)

        # Clear buffer
        self.ser.flushInput()
        self.ser.flushOutput()

    def recieve_packet(self, data_format, data_length):
        checksum = 0
        calcsum = 0
        payload = ''
        rx_data = []

        # Recieve data
        try:
            if self.ser.inWaiting() >= (data_length + 4):
                # Verify header
                if self.ser.read(1).decode('utf-8') != '$':
                    return None
                if self.ser.read(1).decode('utf-8') != '>':
                    return None

                # Verify data length
                data = int(ord(self.ser.read(1).decode('utf-8')))
                if data != data_length:
                    return None

                payload = self.ser.read(data_length)
                checksum = self.ser.read(1)

                # Clear buffer
                self.ser.flushInput()
                self.ser.flushOutput()

                # Verify checksum
                for byte in payload:
                    calcsum ^= byte
                if calcsum != ord(checksum):
                    return None

                # Unpack data
                rx_data = list(struct.unpack(data_format, payload))
                return rx_data

        except Exception as error:
            traceback.print_tb(error.__traceback__)

        # Clear buffer
        self.ser.flushInput()
        self.ser.flushOutput()

        return None

    def send(self):
        data = [
            self.tx_arm,
            self.tx_steering,
            self.tx_throttle,
            self.tx_direction,
        ]

        self.send_packet(data, '<BBBB')

    def recieve(self, delay=0.2, max_retries=5):
        return False

        data = None
        retries = 0
        while not data:
            if retries > max_retries:
                return False
            time.sleep(delay)
            retries += 1
            data = self.recieve_packet('<B', 1)

        self.parse_data(data)
        # self.display()
        return True

    def parse_data(self, data):
        raise NotImplementedError

    def display(self):
        raise NotImplementedError


if __name__ == '__main__':
    packet = Packet()
    packet.open('/dev/ttyACM0')

    # Recieve test
    # while True:
    #     packet.recieve()

    # Send test
    packet.tx_arm = False
    packet.tx_steering = 10
    packet.tx_velocity = 90
    while True:
        packet.send()
        time.sleep(1)

    packet.close()
