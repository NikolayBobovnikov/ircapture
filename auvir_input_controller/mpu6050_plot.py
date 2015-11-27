import sys
import os
import time
import matplotlib.pyplot as plt
import serial
import struct
from ctypes import *
from enum import Enum, unique

Animal = Enum('Animal', 'ant bee cat dog')

class POINT(Structure):
  _fields_ = [("x", c_int), 
              ("y", c_int)]

class MPU6050_MotionData_t(Structure):
  _fields_ = [("Accelerometer_X", c_uint16),
              ("Accelerometer_Y", c_uint16),
              ("Accelerometer_Z", c_uint16),
              ("Temperature",     c_uint16),
              ("Gyroscope_X",     c_uint16),
              ("Gyroscope_Y",     c_uint16),
              ("Gyroscope_Z",     c_uint16),
              ("delta_time",      c_uint16)]

MPU6050_MotionData_t_Size = sys.getsizeof(MPU6050_MotionData_t)

@unique
class UART_Commands(Enum):
    UART_COMMAND_NOT_RECEIVED           = 0
    UART_REQUEST_SEND                   = 1
    UART_REQUEST_STOP                   = 2
    UART_REQUEST_SEND_BYTE              = 3
    UART_REQUEST_SEND_MPU6050_TEST_DATA = 4
    UART_REQUEST_SEND_MPU6050_DATA      = 5
    UART_REQUEST_CALIB_DATA             = 6
    UART_TEST_CONNECTION                = 7
    UART_CONNECTION_FAILURE             = 8
    UART_CONENCTION_OK                  = 9
    UART_MPU6050_TEST_CONNECTION	      = 10
    UART_MPU6050_CONENCTION_FAILURE     = 11
    UART_MPU6050_CONENCTION_OK          = 12
    UART_NULL_RESPONSE                  = 13


print("Size of MPU6050_MotionData_t: ", MPU6050_MotionData_t_Size)

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM7' #com port of avr 
ser.timeout = None
ser.open()        #Opens SerialPort
                
if(not ser.closed):
    print("Opened serial port")
    
try:
    i = 0
    while i < 100:
      os.system("cls");
      i = i + 1
      command = UART_Commands.UART_REQUEST_SEND_MPU6050_DATA
      
      values = (UART_Commands.UART_REQUEST_SEND_MPU6050_DATA)
      s = struct.Struct('I')
      packed_data = s.pack(*values)
      print ('Uses           :', s.size, 'bytes')
      
      ser.write(packed_data)
      time.sleep(0.1) 
      
      
except:
    print("Exception occured, closing serial port")
    ser.close()
  


plt.plot([1,2,3,4])
plt.ylabel('some numbers')
plt.show()
