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
import matplotlib.animation as animation
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
    def __init__(self, serialPort, maxLen):
        # open serial port
        self.ser = serialPort

        self.ax = deque([0.0]*maxLen)
        self.ay = deque([0.0]*maxLen)
        self.maxLen = maxLen
        self.curr_time = 0

        # add to buffer
    def addToBuf(self, buf, val):
        if len(buf) < self.maxLen:
            buf.append(val)
        else:
            buf.pop()
            buf.appendleft(val)

    # add data
    def add(self, data):
        assert(len(data) == 2)
        self.addToBuf(self.ax, data[0])
        self.addToBuf(self.ay, data[1])

    # update plot
    def update(self, frameNum, a0, a1):
        try:
            data = []
            for i in range(0,100):
                adc_raw_val = self.ser.read(size=4)
                self.curr_time += 1
                data.append(self.curr_time)
                adc_val = struct.unpack('i', adc_raw_val)[0]
                data.append(adc_val)

                # print data
                if(len(data) == 2):
                    self.add(data)
                    a0.set_data(range(self.maxLen), self.ax)
                    a1.set_data(range(self.maxLen), self.ay)
        except KeyboardInterrupt:
            print('exiting')

        return a0, 

    # clean up
    def close(self):
        # close serial
        self.ser.flush()
        self.ser.close()   

def animate_adc_values(cdc_device):
    analogPlot = AnalogPlot(cdc_device, 1000)
    # set up animation
    fig = plt.figure()
    ax = plt.axes(xlim=(0, 1000), ylim=(0, 4200))
    a0, = ax.plot([], [])
    a1, = ax.plot([], [])
    print("start animation")
    anim = animation.FuncAnimation(fig, analogPlot.update,
                                   fargs=(a0, a1), interval=1)
    # show plot
    plt.show()
    # clean up
    print('exiting.')


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
        print("port name: [" + sPortName + "]")
        print("port device: [" + sDesc + "]")
        print("port hW: [" + sHw + "]")
        if re.match(".*ttyUSB[0-9]|.*ttyACM[0-9]|COM[0-9]+", sPortName):
            # 'STM32 Virtual ComPort'
            print("port name: [" + sPortName + "]")
            print("port device: [" + sDesc + "]")
            print("port hW: [" + sHw + "]")
            try:
                with serial.Serial(sPortName) as cdc_device:
                    process_serial_device(cdc_device)
                    # animate_adc_values(cdc_device)
            except Exception as e:
                print('ouch! error: ')
                print(e)


def process_serial_device(cdc_device):
    print("start processing serial device")

    if cdc_device.isOpen():
        data = []
        print("port is open: " + cdc_device.name)
        new_data = cdc_device.read(4 * 1000)
        data.extend(struct.unpack('<%dI' % 1000, new_data))
        plt.plot(data)
        plt.show()
    print("stop processing serial device")


if __name__ == "__main__":
    print("version: " + sys.version)
    use_serial()


