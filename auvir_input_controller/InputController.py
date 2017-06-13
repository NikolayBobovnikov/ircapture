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
from scipy import signal
from matplotlib import pyplot as plt
from matplotlib import animation

from time import sleep
from collections import deque
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
        print("port is open: " + cdc_device.name)
        new_data = cdc_device.read(4 * 1000)
        array = np.asarray(struct.unpack('<%dI' % 1000, new_data), dtype=np.int32)
        np.savetxt(fname = "./data.txt", X = array)
        plt.plot(array)
        plt.show()
    print("stop processing serial device")


def moving_average(a, n=3) :
    prefix_array = [0 for x in range(1,n)]
    npprefix = np.asarray(prefix_array)
    print(npprefix)
    ret = np.cumsum(a, dtype=np.int32)
    ret = np.concatenate((npprefix,ret))
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

def get_middle(s, N = 70):
    signal_med = [np.average(s[0:N]) for x in range(0,N)]
    window_signal = s.tolist()[0 : N-1]
    curr_max = max(window_signal)
    curr_min = min(window_signal)

    for i in range (N, 1000):
        prev = window_signal[0]
        window_signal = np.delete(window_signal, [0]).tolist()
        window_signal.append(s[i])
        # maximum = max([curr_max, prev])
        # minimum = min([curr_min, prev])

        # curr_max = max(curr_max, s2[i])
        # curr_min = min(curr_min, s2[i])

        curr_max = max(s[i-N : i])
        curr_min = min(s[i-N : i])

        med = (curr_min + curr_max )/ 2
        signal_med.append(med)
    return signal_med

def analyze_signal():
    s1 = np.loadtxt("./data.txt", dtype=np.int32)
    s2 = signal.medfilt(s1, 5)
    mov_avg = moving_average(s2, 3)
    t = np.arange(0,len(s1),1)

    N = 70
    signal_med = get_middle(s2,70) 

    print("signal med size:")
    print(str(len(signal_med)))

    fig,ax = plt.subplots()
    # fig.figure(1)
    ax.plot(t, s1, "bo", label='signal samples')
    ax.plot(t, s2, "y", label='median filter')
    ax.plot(t, mov_avg, "r", label='avg of median')
    ax.plot(t, signal_med, "g", label='min/max middle - threshold')
    legend = ax.legend(loc='lower center', shadow=True)
    frame = legend.get_frame()
    frame.set_facecolor('0.90')

    plt.show()


if __name__ == "__main__":
    print("version: " + sys.version)    # use_serial()
    # use_serial()
    # analyze_signal()
    cdc_device = serial.Serial('COM9')
    # plt.axis([0,1000,0,4192])
    # axes = plt.gca()
    # axes.set_autoscale_on(False)
    # axes.set_xlim([0,1000])
    # axes.set_ylim([0,4192])

    # Set the limits of the plot
    plt.xlim(0,1000)
    plt.ylim(0,4192)
    plt.autoscale(False)

    plt.ion()
    zero_crossings = []
    while plt.fignum_exists(1) and cdc_device.isOpen():
        plt.clf()
        new_data = cdc_device.read(4 * 1000)
        s1 = np.asarray(struct.unpack('<%dI' % 1000, new_data), dtype=np.int32)
        s2 = signal.medfilt(s1, 5)
        mov_avg = moving_average(s2, 3)
        t = np.arange(0,len(s1),1)
        signal_med = get_middle(s2,70) 

        pwm_received = np.asarray([-1 if signal_med[x] < mov_avg[x] else 1 for x in range(0,1000)])
        zero_crossing = ((pwm_received[:-1] * pwm_received[1:]) < 0).sum()
        zero_crossings.append(zero_crossing)


        is_pwm = [1000 if zero_crossing > 40 and zero_crossing < 60 else -100 for x in range(0,1000)]


        # fig.figure(1)
        plt.plot(t, s1, "bo", label='signal samples')
        # plt.plot(t, s2, "y", label='median filter')
        # plt.plot(t, mov_avg, "r", label='avg of median')
        # plt.plot(t, signal_med, "g", label='min/max middle - threshold')
        plt.plot(t, is_pwm, "g", label='is pwm detected', linewidth = 3)
        legend = plt.legend(loc='lower center', shadow=True)
        frame = legend.get_frame()
        frame.set_facecolor('0.90')
        plt.pause(0.3)
    print(np.mean(zero_crossings))
