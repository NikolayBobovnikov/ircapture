# -*- coding: utf-8 -*-
"""
Created on Thu Mar 31 10:56:50 2016

@author: nbobovnikov
"""

import usb
import usb.core
import usb.util
import usb.control

import sys
import glob
import serial


def serial_ports():
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



print ("List all devices...")
for dev in usb.core.find(find_all=True):
    print ("Device:", dev.filename)
    print ("  idVendor: %d (%s)" % (dev.idVendor, hex(dev.idVendor)))
    print ("  idProduct: %d (%s)" % (dev.idProduct, hex(dev.idProduct)))
    
    
if __name__ == "__main__":
    
    ports = serial_ports()
    print(ports)
    
    if ports:
        #just take first port from the list
        port = ports[0]
        
         #Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
        ser = serial.Serial(port, 9600, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
        for i in range(0,10):
            line = ser.read(6)
            print(line)
        ser.close()
        