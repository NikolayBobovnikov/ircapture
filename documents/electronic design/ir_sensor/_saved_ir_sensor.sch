EESchema Schematic File Version 2
LIBS:ir_sensor-rescue
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
L GND #PWR01
U 1 1 578E393C
P 1150 1100
F 0 "#PWR01" H 1150 850 50  0001 C CNN
F 1 "GND" H 1150 950 50  0000 C CNN
F 2 "" H 1150 1100 50  0000 C CNN
F 3 "" H 1150 1100 50  0000 C CNN
	1    1150 1100
	0    1    1    0   
$EndComp
Text GLabel 5150 1300 2    60   Input ~ 0
I2C_SCL
Text GLabel 5150 1200 2    60   Input ~ 0
I2C_SDA
Text GLabel 1400 1400 0    60   Input ~ 0
I2C_SDA
Text GLabel 1400 1250 0    60   Input ~ 0
I2C_SCL
Text Notes 6250 3550 0    60   ~ 0
 * A8 OUT  CSN \n * A9 OUT  CE  \n * A10 IN  IRQ \n * A4 OUT  MPU5060_AD0 \n * A5 IN   MPU5060_INT \n * C13 OUT LED_ONBOARD \n * A0 OUT  LED_DBG  \n\nA15 tim2\nB3   tim2\nA2   tim2\nA3   tim2\nA6   tim3\nA7   tim3\nB0   tim3\nB1   tim3\nB6   tim4\nB7   tim4\nB8   tim4\nB9   tim4\n\nI2C:\nB10 SCL\nB11 SDA\n\nSPI:\nB13 SCK\nB14 MISO\nB15 MOSI\n\n
Text GLabel 5150 1900 2    60   Input ~ 0
AD0
Text GLabel 2200 1250 2    60   Input ~ 0
AD0
Text GLabel 5150 1800 2    60   Input ~ 0
MPU5060_INT
Text GLabel 2200 1400 2    60   Input ~ 0
MPU5060_INT
NoConn ~ 2200 1100
NoConn ~ 2200 950 
$Comp
L MPU6050_module U3
U 1 1 578E76DC
P 1350 1550
F 0 "U3" H 1800 2300 60  0000 C CNN
F 1 "MPU6050_module" H 1850 1500 60  0000 C CNN
F 2 "footprints:MPU6050_pin_header_straight" H 700 2550 60  0001 C CNN
F 3 "" H 700 2550 60  0000 C CNN
	1    1350 1550
	1    0    0    -1  
$EndComp
$Comp
L SE8R01_module-RESCUE-ir_sensor U4
U 1 1 578E7808
P 1950 2400
F 0 "U4" H 1750 2750 50  0000 L CNN
F 1 "SE8R01_module" H 1500 2100 50  0000 L CNN
F 2 "footprints:SE8R01_SMD" H 1650 2750 50  0001 L CNN
F 3 "" H 2000 2400 60  0000 C CNN
	1    1950 2400
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR02
U 1 1 578E7909
P 1150 2000
F 0 "#PWR02" H 1150 1850 50  0001 C CNN
F 1 "+3V3" H 1150 2140 50  0000 C CNN
F 2 "" H 1150 2000 50  0000 C CNN
F 3 "" H 1150 2000 50  0000 C CNN
	1    1150 2000
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR03
U 1 1 578E792F
P 1150 2300
F 0 "#PWR03" H 1150 2050 50  0001 C CNN
F 1 "GND" H 1150 2150 50  0000 C CNN
F 2 "" H 1150 2300 50  0000 C CNN
F 3 "" H 1150 2300 50  0000 C CNN
	1    1150 2300
	0    1    1    0   
$EndComp
Text GLabel 4300 1200 0    60   Input ~ 0
SE8R01_CSN
Text GLabel 4300 1400 0    60   Input ~ 0
SE8R01_IRQ
Text GLabel 2200 2600 2    60   Input ~ 0
SE8R01_IRQ
Text GLabel 1350 2600 0    60   Input ~ 0
SE8R01_CSN
Text GLabel 1350 2450 0    60   Input ~ 0
SE8R01_CE
Text GLabel 4300 1100 0    60   Input ~ 0
SPI_MOSI
Text GLabel 4300 900  0    60   Input ~ 0
SPI_SCK
Text GLabel 4300 1000 0    60   Input ~ 0
SPI_MISO
Text GLabel 2200 2300 2    60   Input ~ 0
SPI_MOSI
Text GLabel 2200 2150 2    60   Input ~ 0
SPI_SCK
Text GLabel 2200 2450 2    60   Input ~ 0
SPI_MISO
NoConn ~ 4300 1600
NoConn ~ 4300 1900
NoConn ~ 4300 2000
NoConn ~ 5150 2700
NoConn ~ 5150 2600
NoConn ~ 5150 2500
NoConn ~ 5150 2400
NoConn ~ 5150 1100
$Comp
L +3V3 #PWR04
U 1 1 578E9CFC
P 3550 2700
F 0 "#PWR04" H 3550 2550 50  0001 C CNN
F 1 "+3V3" H 3550 2840 50  0000 C CNN
F 2 "" H 3550 2700 50  0000 C CNN
F 3 "" H 3550 2700 50  0000 C CNN
	1    3550 2700
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR05
U 1 1 578E9D22
P 3250 2600
F 0 "#PWR05" H 3250 2350 50  0001 C CNN
F 1 "GND" H 3250 2450 50  0000 C CNN
F 2 "" H 3250 2600 50  0000 C CNN
F 3 "" H 3250 2600 50  0000 C CNN
	1    3250 2600
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 578E9EF8
P 1150 2150
F 0 "C3" H 1175 2250 50  0000 L CNN
F 1 "100uf" H 1175 2050 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1188 2000 50  0001 C CNN
F 3 "" H 1150 2150 50  0000 C CNN
	1    1150 2150
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG06
U 1 1 57913BE4
P 3250 2600
F 0 "#FLG06" H 3250 2695 50  0001 C CNN
F 1 "PWR_FLAG" H 3250 2780 50  0000 C CNN
F 2 "" H 3250 2600 50  0000 C CNN
F 3 "" H 3250 2600 50  0000 C CNN
	1    3250 2600
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG07
U 1 1 57913C36
P 3550 2700
F 0 "#FLG07" H 3550 2795 50  0001 C CNN
F 1 "PWR_FLAG" H 3550 2880 50  0000 C CNN
F 2 "" H 3550 2700 50  0000 C CNN
F 3 "" H 3550 2700 50  0000 C CNN
	1    3550 2700
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR08
U 1 1 5791DCDE
P 4150 2500
F 0 "#PWR08" H 4150 2350 50  0001 C CNN
F 1 "+5V" H 4150 2640 50  0000 C CNN
F 2 "" H 4150 2500 50  0000 C CNN
F 3 "" H 4150 2500 50  0000 C CNN
	1    4150 2500
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR09
U 1 1 5791DD0E
P 1400 950
F 0 "#PWR09" H 1400 800 50  0001 C CNN
F 1 "+5V" H 1400 1090 50  0000 C CNN
F 2 "" H 1400 950 50  0000 C CNN
F 3 "" H 1400 950 50  0000 C CNN
	1    1400 950 
	0    -1   -1   0   
$EndComp
$Comp
L STM32F103HEADER U2
U 1 1 5795C959
P 4550 1850
F 0 "U2" H 4700 3100 60  0000 C CNN
F 1 "STM32F103HEADER" H 4750 800 60  0000 C CNN
F 2 "footprints:STM32F103_header_footprint" H 3900 2850 60  0001 C CNN
F 3 "" H 3900 2850 60  0000 C CNN
	1    4550 1850
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG010
U 1 1 5795CABB
P 4150 2800
F 0 "#FLG010" H 4150 2895 50  0001 C CNN
F 1 "PWR_FLAG" H 4150 2980 50  0000 C CNN
F 2 "" H 4150 2800 50  0000 C CNN
F 3 "" H 4150 2800 50  0000 C CNN
	1    4150 2800
	-1   0    0    1   
$EndComp
NoConn ~ 4300 800 
Text GLabel 4300 1300 0    60   Input ~ 0
SE8R01_CE
Text GLabel 5150 2100 2    60   Input ~ 0
input_1
Text GLabel 5150 2000 2    60   Input ~ 0
input_2
Text GLabel 5150 1700 2    60   Input ~ 0
input_3
Text GLabel 5150 1600 2    60   Input ~ 0
input_4
Text GLabel 5150 1500 2    60   Input ~ 0
input_5
Text GLabel 5150 1400 2    60   Input ~ 0
input_6
Text GLabel 4300 2100 0    60   Input ~ 0
input_7
Text GLabel 4300 2200 0    60   Input ~ 0
input_8
Text GLabel 4300 2300 0    60   Input ~ 0
input_9
Text GLabel 4300 2400 0    60   Input ~ 0
input_10
Text GLabel 4300 1800 0    60   Input ~ 0
input_11
Text GLabel 4300 1700 0    60   Input ~ 0
input_12
Text GLabel 5150 2300 2    60   Input ~ 0
LED_DBG
NoConn ~ 5150 2200
NoConn ~ 4300 1500
Text GLabel 1300 3200 0    60   Input ~ 0
input_1
Text GLabel 1300 3300 0    60   Input ~ 0
input_2
Text GLabel 1300 3400 0    60   Input ~ 0
input_3
Text GLabel 1300 3500 0    60   Input ~ 0
input_4
Text GLabel 1300 3600 0    60   Input ~ 0
input_5
Text GLabel 1300 3700 0    60   Input ~ 0
input_6
Text GLabel 1300 4200 0    60   Input ~ 0
input_7
Text GLabel 1300 4300 0    60   Input ~ 0
input_8
Text GLabel 1300 4400 0    60   Input ~ 0
input_9
Text GLabel 1300 4100 0    60   Input ~ 0
input_11
Text GLabel 1300 4000 0    60   Input ~ 0
input_12
Text GLabel 1300 4500 0    60   Input ~ 0
input_10
Text GLabel 2100 5100 2    60   Input ~ 0
LED_DBG
$Comp
L Conn_01x01 J2
U 1 1 59FB56D0
P 1900 5100
F 0 "J2" H 1900 5200 50  0000 C CNN
F 1 "Conn_01x01" H 1900 5000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 1900 5100 50  0001 C CNN
F 3 "" H 1900 5100 50  0001 C CNN
	1    1900 5100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR011
U 1 1 59FB5965
P 1900 3450
F 0 "#PWR011" H 1900 3200 50  0001 C CNN
F 1 "GND" H 1900 3300 50  0000 C CNN
F 2 "" H 1900 3450 50  0000 C CNN
F 3 "" H 1900 3450 50  0000 C CNN
	1    1900 3450
	0    1    1    0   
$EndComp
$Comp
L Conn_01x06 J3
U 1 1 59FC2749
P 1500 4200
F 0 "J3" H 1500 4500 50  0000 C CNN
F 1 "Conn_01x06" H 1500 3800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 1500 4200 50  0001 C CNN
F 3 "" H 1500 4200 50  0001 C CNN
	1    1500 4200
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06 J5
U 1 1 59FC2827
P 2300 4200
F 0 "J5" H 2300 4500 50  0000 C CNN
F 1 "Conn_01x06" H 2300 3800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 2300 4200 50  0001 C CNN
F 3 "" H 2300 4200 50  0001 C CNN
	1    2300 4200
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06 J7
U 1 1 59FC2879
P 3100 4200
F 0 "J7" H 3100 4500 50  0000 C CNN
F 1 "Conn_01x06" H 3100 3800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 3100 4200 50  0001 C CNN
F 3 "" H 3100 4200 50  0001 C CNN
	1    3100 4200
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06 J1
U 1 1 59FC2A4E
P 1500 3400
F 0 "J1" H 1500 3700 50  0000 C CNN
F 1 "Conn_01x06" H 1500 3000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 1500 3400 50  0001 C CNN
F 3 "" H 1500 3400 50  0001 C CNN
	1    1500 3400
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06 J4
U 1 1 59FC2A54
P 2300 3400
F 0 "J4" H 2300 3700 50  0000 C CNN
F 1 "Conn_01x06" H 2300 3000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 2300 3400 50  0001 C CNN
F 3 "" H 2300 3400 50  0001 C CNN
	1    2300 3400
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06 J6
U 1 1 59FC2A5A
P 3100 3400
F 0 "J6" H 3100 3700 50  0000 C CNN
F 1 "Conn_01x06" H 3100 3000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 3100 3400 50  0001 C CNN
F 3 "" H 3100 3400 50  0001 C CNN
	1    3100 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 1100 1150 1100
Wire Wire Line
	1350 2300 1150 2300
Wire Wire Line
	1350 2150 1350 2000
Wire Wire Line
	1350 2000 1150 2000
Wire Wire Line
	4300 2700 3550 2700
Wire Wire Line
	4300 2600 3250 2600
Wire Wire Line
	4150 2800 4150 2500
Wire Wire Line
	4150 2500 4300 2500
Wire Wire Line
	1900 3700 2100 3700
Wire Wire Line
	1900 3200 1900 3700
Wire Wire Line
	2100 3600 1900 3600
Connection ~ 1900 3600
Wire Wire Line
	2100 3500 1900 3500
Connection ~ 1900 3500
Wire Wire Line
	2100 3400 1900 3400
Connection ~ 1900 3450
Wire Wire Line
	2100 3300 1900 3300
Connection ~ 1900 3400
Wire Wire Line
	2100 3200 1900 3200
Connection ~ 1900 3300
$Comp
L GND #PWR012
U 1 1 59FC2E26
P 1900 4250
F 0 "#PWR012" H 1900 4000 50  0001 C CNN
F 1 "GND" H 1900 4100 50  0000 C CNN
F 2 "" H 1900 4250 50  0000 C CNN
F 3 "" H 1900 4250 50  0000 C CNN
	1    1900 4250
	0    1    1    0   
$EndComp
Wire Wire Line
	1900 4500 2100 4500
Wire Wire Line
	1900 4000 1900 4500
Wire Wire Line
	2100 4400 1900 4400
Connection ~ 1900 4400
Wire Wire Line
	2100 4300 1900 4300
Connection ~ 1900 4300
Wire Wire Line
	2100 4200 1900 4200
Connection ~ 1900 4250
Wire Wire Line
	2100 4100 1900 4100
Connection ~ 1900 4200
Wire Wire Line
	2100 4000 1900 4000
Connection ~ 1900 4100
Wire Wire Line
	2700 3700 2900 3700
Wire Wire Line
	2700 3200 2700 3700
Wire Wire Line
	2900 3600 2700 3600
Connection ~ 2700 3600
Wire Wire Line
	2900 3500 2700 3500
Connection ~ 2700 3500
Wire Wire Line
	2900 3400 2700 3400
Connection ~ 2700 3450
Wire Wire Line
	2900 3300 2700 3300
Connection ~ 2700 3400
Wire Wire Line
	2900 3200 2700 3200
Connection ~ 2700 3300
Wire Wire Line
	2700 4500 2900 4500
Wire Wire Line
	2700 4000 2700 4500
Wire Wire Line
	2900 4400 2700 4400
Connection ~ 2700 4400
Wire Wire Line
	2900 4300 2700 4300
Connection ~ 2700 4300
Wire Wire Line
	2900 4200 2700 4200
Connection ~ 2700 4250
Wire Wire Line
	2900 4100 2700 4100
Connection ~ 2700 4200
Wire Wire Line
	2900 4000 2700 4000
Connection ~ 2700 4100
$Comp
L +3V3 #PWR013
U 1 1 59FC2F1A
P 2700 3450
F 0 "#PWR013" H 2700 3300 50  0001 C CNN
F 1 "+3V3" H 2700 3590 50  0000 C CNN
F 2 "" H 2700 3450 50  0000 C CNN
F 3 "" H 2700 3450 50  0000 C CNN
	1    2700 3450
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR014
U 1 1 59FC2FFF
P 2700 4250
F 0 "#PWR014" H 2700 4100 50  0001 C CNN
F 1 "+3V3" H 2700 4390 50  0000 C CNN
F 2 "" H 2700 4250 50  0000 C CNN
F 3 "" H 2700 4250 50  0000 C CNN
	1    2700 4250
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
