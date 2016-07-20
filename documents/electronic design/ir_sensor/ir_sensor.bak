EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:beamer
LIBS:ir_sensor-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L C C1
U 1 1 578E28E6
P 1600 950
F 0 "C1" H 1625 1050 50  0000 L CNN
F 1 "104" H 1625 850 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1638 800 50  0001 C CNN
F 3 "" H 1600 950 50  0000 C CNN
	1    1600 950 
	1    0    0    -1  
$EndComp
Connection ~ 1600 800 
Wire Wire Line
	1450 1100 2100 1100
Connection ~ 1600 1100
Connection ~ 1850 800 
$Comp
L +3V3 #PWR01
U 1 1 578E2CEF
P 2050 800
F 0 "#PWR01" H 2050 650 50  0001 C CNN
F 1 "+3V3" H 2050 940 50  0000 C CNN
F 2 "" H 2050 800 50  0000 C CNN
F 3 "" H 2050 800 50  0000 C CNN
	1    2050 800 
	0    1    1    0   
$EndComp
$Comp
L GND #PWR02
U 1 1 578E2D11
P 2100 1100
F 0 "#PWR02" H 2100 850 50  0001 C CNN
F 1 "GND" H 2100 950 50  0000 C CNN
F 2 "" H 2100 1100 50  0000 C CNN
F 3 "" H 2100 1100 50  0000 C CNN
	1    2100 1100
	0    -1   -1   0   
$EndComp
Connection ~ 1850 1100
$Comp
L +3V3 #PWR03
U 1 1 578E38B9
P 3400 800
F 0 "#PWR03" H 3400 650 50  0001 C CNN
F 1 "+3V3" H 3400 940 50  0000 C CNN
F 2 "" H 3400 800 50  0000 C CNN
F 3 "" H 3400 800 50  0000 C CNN
	1    3400 800 
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR04
U 1 1 578E393C
P 3400 1100
F 0 "#PWR04" H 3400 850 50  0001 C CNN
F 1 "GND" H 3400 950 50  0000 C CNN
F 2 "" H 3400 1100 50  0000 C CNN
F 3 "" H 3400 1100 50  0000 C CNN
	1    3400 1100
	0    1    1    0   
$EndComp
Wire Wire Line
	3650 1100 3400 1100
Wire Wire Line
	3650 950  3650 800 
Wire Wire Line
	3650 800  3400 800 
Text GLabel 1100 3200 0    60   Input ~ 0
I2C_SCL
Text GLabel 1100 3300 0    60   Input ~ 0
I2C_SDA
Text GLabel 3650 1400 0    60   Input ~ 0
I2C_SDA
Text GLabel 3650 1250 0    60   Input ~ 0
I2C_SCL
Text Notes 5550 1900 0    60   ~ 0
    I2C1:\nPB8     ------> I2C1_SCL\nPB9     ------> I2C1_SDA\n\n    SPI1:\nMISO     ------>A7\nMOSI     ------>A6\nSCK      ------>A5\n\n    SE8R01:\nCSN     ------> A11\nCE       ------> A10\nIRQ      ------> B0
Text GLabel 1950 3000 2    60   Input ~ 0
AD0
Text GLabel 4450 1250 2    60   Input ~ 0
AD0
Text GLabel 1950 3100 2    60   Input ~ 0
MPU5060_INT
Text GLabel 4450 1400 2    60   Input ~ 0
MPU5060_INT
NoConn ~ 4450 1100
NoConn ~ 4450 950 
$Comp
L MPU6050_module U3
U 1 1 578E76DC
P 3600 1550
F 0 "U3" H 4050 2300 60  0000 C CNN
F 1 "MPU6050_module" H 4100 1500 60  0000 C CNN
F 2 "footprints:MPU6050_pin_header_straight" H 2950 2550 60  0001 C CNN
F 3 "" H 2950 2550 60  0000 C CNN
	1    3600 1550
	1    0    0    -1  
$EndComp
$Comp
L SE8R01_module U4
U 1 1 578E7808
P 4200 2250
F 0 "U4" H 4000 2700 50  0000 L CNN
F 1 "SE8R01_module" H 3750 1950 50  0000 L CNN
F 2 "footprints:SE8R01" H 3900 2600 50  0001 L CNN
F 3 "" H 4250 2250 60  0000 C CNN
	1    4200 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 2150 3400 2150
Wire Wire Line
	3600 2000 3600 1850
Wire Wire Line
	3600 1850 3400 1850
$Comp
L +3V3 #PWR05
U 1 1 578E7909
P 3400 1850
F 0 "#PWR05" H 3400 1700 50  0001 C CNN
F 1 "+3V3" H 3400 1990 50  0000 C CNN
F 2 "" H 3400 1850 50  0000 C CNN
F 3 "" H 3400 1850 50  0000 C CNN
	1    3400 1850
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR06
U 1 1 578E792F
P 3400 2150
F 0 "#PWR06" H 3400 1900 50  0001 C CNN
F 1 "GND" H 3400 2000 50  0000 C CNN
F 2 "" H 3400 2150 50  0000 C CNN
F 3 "" H 3400 2150 50  0000 C CNN
	1    3400 2150
	0    1    1    0   
$EndComp
Text GLabel 1100 2400 0    60   Input ~ 0
SE8R01_CSN
Text GLabel 1100 2300 0    60   Input ~ 0
SE8R01_CE
Text GLabel 1950 2400 2    60   Input ~ 0
SE8R01_IRQ
Text GLabel 4450 2450 2    60   Input ~ 0
SE8R01_IRQ
Text GLabel 3600 2450 0    60   Input ~ 0
SE8R01_CSN
Text GLabel 3600 2300 0    60   Input ~ 0
SE8R01_CE
Text GLabel 1950 2600 2    60   Input ~ 0
SPI_MOSI
Text GLabel 1950 2700 2    60   Input ~ 0
SPI_SCK
Text GLabel 1950 2500 2    60   Input ~ 0
SPI_MISO
Text GLabel 4450 2150 2    60   Input ~ 0
SPI_MOSI
Text GLabel 4450 2000 2    60   Input ~ 0
SPI_SCK
Text GLabel 4450 2300 2    60   Input ~ 0
SPI_MISO
Text GLabel 1450 1250 2    60   Input ~ 0
IR_RCV
Text GLabel 1100 2100 0    60   Input ~ 0
IR_RCV
NoConn ~ 1100 1700
NoConn ~ 1100 1800
NoConn ~ 1100 1900
NoConn ~ 1100 2000
NoConn ~ 1100 2200
NoConn ~ 1100 2500
NoConn ~ 1100 2600
NoConn ~ 1100 2700
NoConn ~ 1100 2800
NoConn ~ 1100 2900
NoConn ~ 1100 3400
NoConn ~ 1950 3600
NoConn ~ 1950 3500
NoConn ~ 1950 3400
NoConn ~ 1950 3300
NoConn ~ 1950 3200
NoConn ~ 1100 3100
NoConn ~ 1100 3000
NoConn ~ 1950 2900
NoConn ~ 1950 2800
NoConn ~ 1950 2300
NoConn ~ 1950 2200
NoConn ~ 1950 2100
NoConn ~ 1950 2000
$Comp
L +3V3 #PWR07
U 1 1 578E9CFC
P 1100 3600
F 0 "#PWR07" H 1100 3450 50  0001 C CNN
F 1 "+3V3" H 1100 3740 50  0000 C CNN
F 2 "" H 1100 3600 50  0000 C CNN
F 3 "" H 1100 3600 50  0000 C CNN
	1    1100 3600
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR08
U 1 1 578E9D22
P 1100 3500
F 0 "#PWR08" H 1100 3250 50  0001 C CNN
F 1 "GND" H 1100 3350 50  0000 C CNN
F 2 "" H 1100 3500 50  0000 C CNN
F 3 "" H 1100 3500 50  0000 C CNN
	1    1100 3500
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 578E9EF8
P 3400 950
F 0 "C3" H 3425 1050 50  0000 L CNN
F 1 "100uf" H 3425 850 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3438 800 50  0001 C CNN
F 3 "" H 3400 950 50  0000 C CNN
	1    3400 950 
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 578EA087
P 3400 2000
F 0 "C4" H 3425 2100 50  0000 L CNN
F 1 "100uf" H 3425 1900 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3438 1850 50  0001 C CNN
F 3 "" H 3400 2000 50  0000 C CNN
	1    3400 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 950  1450 800 
Wire Wire Line
	1450 800  2050 800 
$Comp
L TL1838 U1
U 1 1 578E371A
P 950 1350
F 0 "U1" H 1150 1850 60  0000 C CNN
F 1 "TL1838" H 1150 1300 60  0000 C CNN
F 2 "footprints:TL1838_footprint" H 300 2350 60  0001 C CNN
F 3 "" H 300 2350 60  0000 C CNN
	1    950  1350
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 578EB127
P 1850 950
F 0 "C2" H 1875 1050 50  0000 L CNN
F 1 "100uf" H 1875 850 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1888 800 50  0001 C CNN
F 3 "" H 1850 950 50  0000 C CNN
	1    1850 950 
	1    0    0    -1  
$EndComp
$Comp
L STM32F103HEADER U?
U 1 1 578E6D6B
P 1350 2750
F 0 "U?" H 1500 4000 60  0000 C CNN
F 1 "STM32F103HEADER" H 1550 1700 60  0000 C CNN
F 2 "" H 700 3750 60  0000 C CNN
F 3 "" H 700 3750 60  0000 C CNN
	1    1350 2750
	1    0    0    -1  
$EndComp
$EndSCHEMATC
