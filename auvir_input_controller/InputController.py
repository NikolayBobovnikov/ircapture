# -*- coding: utf-8 -*-
"""
Created on Thu Mar 31 10:56:50 2016

@author: nbobovnikov
"""

import serial
import serial.tools.list_ports
# system libraries
import sys
from time import sleep
import glob

import struct
from struct import *


def get_serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def use_serial():
    print ("List all serial ports other way using serial library:")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("There is no available serial ports")
        exit


    #Get correct port name. Need to identify among others if there are many
    #just use first port from the list, using one of 2 options below. TODO: fix that, determine required port somehow
    #port_name = port_names[0]


    target_port_name = "COM9"

    print("scanning ports")
    for port_name in ports:
        if port_name.device == "COM9":
            print("opening " + target_port_name)
            print("device: " + port_name.device)

            # Tried with and without the last 3 parameters, and also at 1Mbps,
            # same happens.
            try:
                with serial.Serial(port_name.device) as cdc_device:
                    process_serial_device(cdc_device)
            except serial.serialutil.SerialException as e:
                print(e)

def process_serial_device(cdc_device):
    print("start processing serial device")
    #formats for unpacking data from received C structures
    msg_formats = {"IMUData" : "3c9H10c",
                   "BeamerData" : "3ccHcHcHcHcHcHcHcHcH" }
    if cdc_device.isOpen():
        print("COM port is open")
        #data = cdc_device.read(32)
        #line = cdc_device.readline()
        data = cdc_device.read(1)
        print(ord(data))
        # decoded_Data = unpack(msg_formats["IMUData"], data)
        # print(decoded_Data)
    print("stop processing serial device")


if __name__ == "__main__":
    print("hello!")
    use_serial()
