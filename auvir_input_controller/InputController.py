# -*- coding: utf-8 -*-
"""
Created on Thu Mar 31 10:56:50 2016

@author: nbobovnikov
"""

import serial
from serial.tools import list_ports
# system libraries
import sys
import glob
import re

import struct
from struct import *
import numpy as np
from time import sleep
from collections import deque
from matplotlib import pyplot as plt
import math

# class that holds analog data for N samples
class AnalogData:
    # constr
    def __init__(self, maxLen):
        self.ax = deque([0.0]*maxLen)
        self.maxLen = maxLen

    # ring buffer
    def addToBuf(self, buf, val):
        if len(buf) < self.maxLen:
            buf.append(val)
        else:
            buf.pop()
            buf.appendleft(val)

    # add data
    def add(self, data):
        assert(len(data) == 1)
        self.addToBuf(self.ax, data[0])


# plot class
class AnalogPlot:
    # constr
    def __init__(self, analogData):
        # set plot to animated
        plt.ion() 
        self.axline, = plt.plot(analogData.ax)
        plt.ylim([0, 400])

        # update plot
    def update(self, analogData):
        self.axline.set_ydata(analogData.ax)
        plt.draw()


# main() function
def realtime_plot():
    # plot parameters
    analogData = AnalogData(100)
    analogPlot = AnalogPlot(analogData)

    # open serial port
    #ser = serial.Serial(strPort, 9600)
    ports = list(serial.tools.list_ports.comports())
    sDesc = ""
    sPortName = ""
    for port_name in ports:
        sPortName = port_name[0]
        sDesc = port_name[1]
        print("port name: [" + sDesc + "]")
        if sDesc == 'STM32 Virtual ComPort':
            continue;

    with serial.Serial(sPortName) as ser:
        while True:
            try:
                #line = ser.readline()
                try:
                    # data = [float(val) for val in line.split()]
                    for i in range(0,10):
                        analogDAta.add(ser.read(1))
                    analogPlot.update(analogData)
                except:
                    pass
            except KeyboardInterrupt:
                print ('exiting')
                break


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
    for port in ListPortInfoports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def use_serial():
    ports = list(list_ports.comports())
    if not ports:
        print("There is no available serial ports")
        exit

    #Get correct port name. Need to identify among others2 if there are many
    #just use first port from the list, using one of 2 options below. TODO: fix that, determine required port somehow
    #port_name = port_names[0]

    print("scanning ports")
    for port_name in ports:
        sPortName = port_name[0]
        sDesc = port_name[1]
        sHw = port_name[2]
        # if sDesc == 'STM32 Virtual ComPort':
        if re.match(".*ttyUSB[0-9]|.*ttyACM[0-9]", sPortName):
            # 'STM32 Virtual ComPort'
            print("port name: [" + sPortName + "]")
            print("port device: [" + sDesc + "]")
            print("port hW: [" + sHw + "]")
            try:
                with serial.Serial(sPortName) as cdc_device:
                    process_serial_device(cdc_device)
            except:
                print('failed to open ' + sPortName + '; try again with ' + sDesc)
                try:
                    with serial.Serial(sDesc) as cdc_device:
                        process_serial_device(cdc_device)
                except serial.serialutil.SerialException as e:
                    print(e)


def process_serial_device(cdc_device):
    print("start processing serial device")
    #formats for unpacking data from received C structures
    msg_formats = {"IMUData" : "3c9H10c",
                   "BeamerData" : "3ccHcHcHcHcHcHcHcHcH" }

    data = []
    if cdc_device.isOpen():
        print("port is open: " + cdc_device.name)
        for i in range(0,3000):
            # data = cdc_device.read(32)
            #line = cdc_device.readline()
            new_data = cdc_device.read(4)
            # print('int from bytes: ' + str(int.from_bytes(data, byteorder='big')))
            # print('str(): ' + str(data))
            # print(struct.unpack('i', data))
            data.append(struct.unpack('i', new_data)[0])
    cdc_device.close()
    plt.plot(data)
    plt.show()
            # decoded_Data = unpack(msg_formats["IMUData"], data)
            # print(decoded_Data)
    print("stop processing serial device")


if __name__ == "__main__":
    print("version: " + sys.version)
    use_serial()
    # analogData = AnalogData(100)
    # analogPlot = AnalogPlot(analogData)


