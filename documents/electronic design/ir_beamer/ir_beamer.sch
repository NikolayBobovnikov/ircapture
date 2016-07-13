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
LIBS:2N7002K
LIBS:beamer
LIBS:ir_beamer-cache
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
L STM32F103_header P1
U 1 1 5786C27B
P 3200 2250
F 0 "P1" H 3600 3300 50  0000 C CNN
F 1 "STM32F103_header" V 3550 2600 50  0000 C CNN
F 2 "" H 3200 2250 50  0001 C CNN
F 3 "" H 3200 2250 50  0000 C CNN
	1    3200 2250
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 5786C38B
P 2300 1800
F 0 "D1" H 2300 1900 50  0000 C CNN
F 1 "LED" H 2300 1700 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 2300 1800 50  0001 C CNN
F 3 "" H 2300 1800 50  0000 C CNN
	1    2300 1800
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5786C41A
P 2650 1800
F 0 "R1" V 2730 1800 50  0000 C CNN
F 1 "R" V 2650 1800 50  0000 C CNN
F 2 "" V 2580 1800 50  0001 C CNN
F 3 "" H 2650 1800 50  0000 C CNN
	1    2650 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	2800 1800 3000 1800
Wire Wire Line
	2100 1800 2050 1800
Wire Wire Line
	2050 1800 2050 3100
Wire Wire Line
	2050 3100 3000 3100
NoConn ~ 3000 1300
NoConn ~ 3000 1400
NoConn ~ 3000 1500
NoConn ~ 3000 1600
NoConn ~ 3000 1700
NoConn ~ 3000 1900
NoConn ~ 3000 2000
NoConn ~ 3000 2100
NoConn ~ 3000 2200
NoConn ~ 3000 2300
NoConn ~ 3000 2400
NoConn ~ 3000 2500
NoConn ~ 3000 2600
NoConn ~ 3000 2700
NoConn ~ 3000 2800
NoConn ~ 3000 2900
NoConn ~ 3000 3000
NoConn ~ 3000 3200
NoConn ~ 4150 3200
NoConn ~ 4150 3100
NoConn ~ 4150 3000
NoConn ~ 4150 2900
NoConn ~ 4150 2800
NoConn ~ 4150 2700
NoConn ~ 4150 2600
NoConn ~ 4150 2500
NoConn ~ 4150 2400
NoConn ~ 4150 2300
NoConn ~ 4150 2200
NoConn ~ 4150 2100
NoConn ~ 4150 2000
NoConn ~ 4150 1900
NoConn ~ 4150 1800
NoConn ~ 4150 1700
NoConn ~ 4150 1600
NoConn ~ 4150 1500
NoConn ~ 4150 1400
NoConn ~ 4150 1300
$EndSCHEMATC
