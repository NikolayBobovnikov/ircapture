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
import serial.tools.list_ports


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

	
def process_serial_device(device):
    if cdc_device.isOpen():
        data_to_write = "Hello!"
        cdc_device.write(data_to_write)
        cdc_device.write(data_to_write)
        #line = cdc_device.read(7)
        #print(line)
            
            
			
    
if __name__ == "__main__":
    
    print ("List all devices...")
    for dev in usb.core.find(find_all=True):
        print ("  idVendor: %d (%s)" % (dev.idVendor, hex(dev.idVendor)))
        print ("  idProduct: %d (%s)" % (dev.idProduct, hex(dev.idProduct)))
    
    
    print ("List all serial ports...")
    port_names = serial_ports()
    print ("List all serial ports other way...")
    ports = list(serial.tools.list_ports.comports())
    
    #check that there are ports available
    if not port_names:
        print("There is no available serial ports")
        exit
    if not ports:
        print("There is no available serial ports")
        exit
    
    #Get correct port name. Need to identify among others if there are many
    #just use first port from the list, using one of 2 options below. TODO: fix that, determine required port somehow
    #port_name = port_names[0]
    port_name = ports[0].device
    print(port_name)
    
     #Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
    try:
        with serial.Serial(port_name) as cdc_device:
            process_serial_device(cdc_device)
    except serial.serialutil.SerialException as e:
        print(e)
        
    
    
 
        
        