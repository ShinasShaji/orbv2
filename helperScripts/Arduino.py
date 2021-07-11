"""Module containing class Arduino to handle connections to Arduino COM and log data"""

import os
import time

import serial
from serial.serialutil import SerialTimeoutException


class Arduino:
    """Class to handle connections to Arduino COM and log data to defined csv files"""
    # Connection
    arduino = None
    baudrate = 9600
    connected = None
    timeout = 2
    interval = 0.2    

    # Use different port name list for connection based on platform
    # Linux
    if os.name == 'posix':
        ports = ['/dev/ttyS0', '/dev/ttyS1']
    # Windows
    elif os.name == 'nt':
        ports = ['COM8', 'COM6']


    def __init__(self):
        # Connection 
        self.arduino = serial.Serial()
        self.arduino.baudrate = self.baudrate
        self.connected = False


    def attemptPortConnection(self, port):
        """Attempt to connect on the passed port"""
        count = 0
        self.connected = False
        self.arduino.port = port
        print(''.join(['\nConnecting on port ', port, '...\n']))
        try:
            self.arduino.open()
            self.arduino.flushInput()

            while not self.connected:
                self.arduino.write(b'R')
                if self.arduino.in_waiting:
                    print(''.join(['Connected on port ', port, '.']))
                    self.connected = True
                    return True

                time.sleep(self.interval)
                count += 1
                if count * self.interval > self.timeout:
                    raise SerialTimeoutException(''.join(['Could not connect on ', port, ' within timeout interval.']))
        except:
            print(''.join(['Could not connect on port ', port, '.']))
            self.connected = False
            return False


    def attemptConnection(self):
        """Attempt to connect on all specified ports, tries the next if one fails. Uses attemptPortConnection to do so, internally."""
        for port in self.ports:
            if (self.attemptPortConnection(port)):
                time.sleep(1)
                return True
        print('Could not connect on any port.')
        return False
     

    def closeConnection(self):
        """Close connections and files, and exit"""
        print('Cleaning up and closing files...\n')
        try:
            self.arduino.close()
            print ('Done!\n')
        except Exception as e:
            print (e)
            print ('Something went wrong when wrapping up. Closing down anyway...')
