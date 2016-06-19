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
LIBS:NRF24L01-BRKOUT
LIBS:conn_R
LIBS:2N7002K
LIBS:74HC595D
LIBS:beamer-cache
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
L CONN_01X20 P1
U 1 1 5713A28F
P 2600 4550
F 0 "P1" H 2600 5600 50  0000 C CNN
F 1 "CONN_01X20" V 2700 4550 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x20" H 2600 4550 50  0001 C CNN
F 3 "" H 2600 4550 50  0000 C CNN
	1    2600 4550
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X20 P2
U 1 1 5713A2E5
P 4750 4550
F 0 "P2" H 4750 5600 50  0000 C CNN
F 1 "CONN_01X20" V 4850 4550 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x20" H 4750 4550 50  0001 C CNN
F 3 "" H 4750 4550 50  0000 C CNN
	1    4750 4550
	1    0    0    -1  
$EndComp
Text GLabel 9550 1700 2    60   Input ~ 0
SPI1_MOSI
Text GLabel 9550 1900 2    60   Input ~ 0
SPI1_MISO
Text GLabel 9550 2100 2    60   Input ~ 0
NRF_CSN1
Text GLabel 8150 1900 0    60   Input ~ 0
NRF_IRQ1
Text GLabel 8150 2100 0    60   Input ~ 0
NRF_CE1
Text GLabel 9550 1500 2    60   Input ~ 0
SPI1_SCK
$Comp
L LED D1
U 1 1 5713A42A
P 1450 1300
F 0 "D1" H 1450 1400 50  0000 C CNN
F 1 "LED" H 1450 1200 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1450 1300 50  0001 C CNN
F 3 "" H 1450 1300 50  0000 C CNN
	1    1450 1300
	0    1    -1   0   
$EndComp
$Comp
L 2N7002K T1
U 1 1 5713C230
P 1350 1900
F 0 "T1" H 1150 2150 50  0000 L CNN
F 1 "2N7002K" H 1000 2050 50  0000 L CNN
F 2 "sot23_modified:SOT23Mod" H 1350 1900 50  0001 L CNN
F 3 "" H 1350 1900 60  0000 C CNN
	1    1350 1900
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR01
U 1 1 5713DE3B
P 2950 750
F 0 "#PWR01" H 2950 600 50  0001 C CNN
F 1 "+3.3V" H 2950 890 50  0000 C CNN
F 2 "" H 2950 750 50  0000 C CNN
F 3 "" H 2950 750 50  0000 C CNN
	1    2950 750 
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5713DE5D
P 2950 950
F 0 "R1" V 3030 950 50  0000 C CNN
F 1 "R" V 2950 950 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2880 950 50  0001 C CNN
F 3 "" H 2950 950 50  0000 C CNN
	1    2950 950 
	1    0    0    -1  
$EndComp
$Comp
L 2N7002K T4
U 1 1 5713DF4B
P 2850 2800
F 0 "T4" H 2650 3050 50  0000 L CNN
F 1 "2N7002K" H 2500 2950 50  0000 L CNN
F 2 "sot23_modified:SOT23Mod" H 2850 2800 50  0001 L CNN
F 3 "" H 2850 2800 60  0000 C CNN
	1    2850 2800
	1    0    0    -1  
$EndComp
Text GLabel 1050 1900 3    60   Input ~ 0
IR_PIN_1
Text GLabel 2550 2800 0    60   Input ~ 0
PWM_38K
$Comp
L LED D2
U 1 1 5713E142
P 2050 1300
F 0 "D2" H 2050 1400 50  0000 C CNN
F 1 "LED" H 2050 1200 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 2050 1300 50  0001 C CNN
F 3 "" H 2050 1300 50  0000 C CNN
	1    2050 1300
	0    1    -1   0   
$EndComp
$Comp
L 2N7002K T2
U 1 1 5713E148
P 1950 1900
F 0 "T2" H 1750 2150 50  0000 L CNN
F 1 "2N7002K" H 1600 2050 50  0000 L CNN
F 2 "sot23_modified:SOT23Mod" H 1950 1900 50  0001 L CNN
F 3 "" H 1950 1900 60  0000 C CNN
	1    1950 1900
	1    0    0    -1  
$EndComp
Text GLabel 1650 1900 3    60   Input ~ 0
IR_PIN_2
$Comp
L LED D3
U 1 1 5713E257
P 2650 1300
F 0 "D3" H 2650 1400 50  0000 C CNN
F 1 "LED" H 2650 1200 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 2650 1300 50  0001 C CNN
F 3 "" H 2650 1300 50  0000 C CNN
	1    2650 1300
	0    1    -1   0   
$EndComp
$Comp
L 2N7002K T3
U 1 1 5713E25D
P 2550 1900
F 0 "T3" H 2350 2150 50  0000 L CNN
F 1 "2N7002K" H 2200 2050 50  0000 L CNN
F 2 "sot23_modified:SOT23Mod" H 2550 1900 50  0001 L CNN
F 3 "" H 2550 1900 60  0000 C CNN
	1    2550 1900
	1    0    0    -1  
$EndComp
Text GLabel 2250 1900 3    60   Input ~ 0
IR_PIN_3
$Comp
L LED D4
U 1 1 5713E26B
P 3250 1300
F 0 "D4" H 3250 1400 50  0000 C CNN
F 1 "LED" H 3250 1200 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 3250 1300 50  0001 C CNN
F 3 "" H 3250 1300 50  0000 C CNN
	1    3250 1300
	0    1    -1   0   
$EndComp
$Comp
L 2N7002K T5
U 1 1 5713E271
P 3150 1900
F 0 "T5" H 2950 2150 50  0000 L CNN
F 1 "2N7002K" H 2800 2050 50  0000 L CNN
F 2 "sot23_modified:SOT23Mod" H 3150 1900 50  0001 L CNN
F 3 "" H 3150 1900 60  0000 C CNN
	1    3150 1900
	1    0    0    -1  
$EndComp
Text GLabel 2850 1900 3    60   Input ~ 0
IR_PIN_4
$Comp
L LED D5
U 1 1 5713E4A0
P 3850 1300
F 0 "D5" H 3850 1400 50  0000 C CNN
F 1 "LED" H 3850 1200 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 3850 1300 50  0001 C CNN
F 3 "" H 3850 1300 50  0000 C CNN
	1    3850 1300
	0    1    -1   0   
$EndComp
$Comp
L 2N7002K T6
U 1 1 5713E4A6
P 3750 1900
F 0 "T6" H 3550 2150 50  0000 L CNN
F 1 "2N7002K" H 3400 2050 50  0000 L CNN
F 2 "sot23_modified:SOT23Mod" H 3750 1900 50  0001 L CNN
F 3 "" H 3750 1900 60  0000 C CNN
	1    3750 1900
	1    0    0    -1  
$EndComp
Text GLabel 3450 1900 3    60   Input ~ 0
IR_PIN_5
$Comp
L LED D6
U 1 1 5713E4AE
P 4450 1300
F 0 "D6" H 4450 1400 50  0000 C CNN
F 1 "LED" H 4450 1200 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 4450 1300 50  0001 C CNN
F 3 "" H 4450 1300 50  0000 C CNN
	1    4450 1300
	0    1    -1   0   
$EndComp
$Comp
L 2N7002K T7
U 1 1 5713E4B4
P 4350 1900
F 0 "T7" H 4150 2150 50  0000 L CNN
F 1 "2N7002K" H 4000 2050 50  0000 L CNN
F 2 "sot23_modified:SOT23Mod" H 4350 1900 50  0001 L CNN
F 3 "" H 4350 1900 60  0000 C CNN
	1    4350 1900
	1    0    0    -1  
$EndComp
Text GLabel 4050 1900 3    60   Input ~ 0
IR_PIN_6
$Comp
L LED D7
U 1 1 5713E4BC
P 5050 1300
F 0 "D7" H 5050 1400 50  0000 C CNN
F 1 "LED" H 5050 1200 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 5050 1300 50  0001 C CNN
F 3 "" H 5050 1300 50  0000 C CNN
	1    5050 1300
	0    1    -1   0   
$EndComp
$Comp
L 2N7002K T8
U 1 1 5713E4C2
P 4950 1900
F 0 "T8" H 4750 2150 50  0000 L CNN
F 1 "2N7002K" H 4600 2050 50  0000 L CNN
F 2 "sot23_modified:SOT23Mod" H 4950 1900 50  0001 L CNN
F 3 "" H 4950 1900 60  0000 C CNN
	1    4950 1900
	1    0    0    -1  
$EndComp
Text GLabel 4650 1900 3    60   Input ~ 0
IR_PIN_7
$Comp
L LED D8
U 1 1 5713E4CA
P 5650 1300
F 0 "D8" H 5650 1400 50  0000 C CNN
F 1 "LED" H 5650 1200 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 5650 1300 50  0001 C CNN
F 3 "" H 5650 1300 50  0000 C CNN
	1    5650 1300
	0    1    -1   0   
$EndComp
$Comp
L 2N7002K T9
U 1 1 5713E4D0
P 5550 1900
F 0 "T9" H 5350 2150 50  0000 L CNN
F 1 "2N7002K" H 5200 2050 50  0000 L CNN
F 2 "sot23_modified:SOT23Mod" H 5550 1900 50  0001 L CNN
F 3 "" H 5550 1900 60  0000 C CNN
	1    5550 1900
	1    0    0    -1  
$EndComp
Text GLabel 5250 1900 3    60   Input ~ 0
IR_PIN_8
$Comp
L LED D9
U 1 1 5713F1E4
P 6900 1300
F 0 "D9" H 6900 1400 50  0000 C CNN
F 1 "LED" H 6900 1200 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 6900 1300 50  0001 C CNN
F 3 "" H 6900 1300 50  0000 C CNN
	1    6900 1300
	0    1    -1   0   
$EndComp
$Comp
L 2N7002K T10
U 1 1 5713F1EA
P 6800 1900
F 0 "T10" H 6600 2150 50  0000 L CNN
F 1 "2N7002K" H 6450 2050 50  0000 L CNN
F 2 "sot23_modified:SOT23Mod" H 6800 1900 50  0001 L CNN
F 3 "" H 6800 1900 60  0000 C CNN
	1    6800 1900
	1    0    0    -1  
$EndComp
Text GLabel 6500 1900 3    60   Input ~ 0
IR_PIN_ID
$Comp
L GND #PWR02
U 1 1 5713F325
P 2950 3100
F 0 "#PWR02" H 2950 2850 50  0001 C CNN
F 1 "GND" H 2950 2950 50  0000 C CNN
F 2 "" H 2950 3100 50  0000 C CNN
F 3 "" H 2950 3100 50  0000 C CNN
	1    2950 3100
	1    0    0    -1  
$EndComp
Text GLabel 4450 4600 0    60   Input ~ 0
SPI1_SCK
Text GLabel 4450 4500 0    60   Input ~ 0
SPI1_MOSI
Text GLabel 4450 4400 0    60   Input ~ 0
SPI1_MISO
Text GLabel 4450 4700 0    60   Input ~ 0
NRF_CSN1
Text GLabel 4450 4800 0    60   Input ~ 0
NRF_CE1
Text GLabel 4450 4300 0    60   Input ~ 0
NRF_IRQ1
Text GLabel 8150 3700 2    60   Input ~ 0
IR_PIN_1
Text GLabel 8150 3800 2    60   Input ~ 0
IR_PIN_2
Text GLabel 8150 3900 2    60   Input ~ 0
IR_PIN_3
Text GLabel 8150 4000 2    60   Input ~ 0
IR_PIN_4
Text GLabel 8150 4100 2    60   Input ~ 0
IR_PIN_5
Text GLabel 8150 4200 2    60   Input ~ 0
IR_PIN_6
Text GLabel 8150 4300 2    60   Input ~ 0
IR_PIN_7
Text GLabel 8150 4400 2    60   Input ~ 0
IR_PIN_8
Text GLabel 2900 5100 2    60   Input ~ 0
IR_PIN_ID
NoConn ~ 2800 5300
NoConn ~ 2800 4200
NoConn ~ 2800 4100
NoConn ~ 2800 4000
NoConn ~ 2800 3900
NoConn ~ 2800 3800
NoConn ~ 2800 3700
NoConn ~ 4550 5500
NoConn ~ 4550 5400
NoConn ~ 4550 5000
NoConn ~ 4550 4100
NoConn ~ 4550 4000
NoConn ~ 4550 3900
$Comp
L GND #PWR03
U 1 1 57141D41
P 4450 3600
F 0 "#PWR03" H 4450 3350 50  0001 C CNN
F 1 "GND" H 4450 3450 50  0000 C CNN
F 2 "" H 4450 3600 50  0000 C CNN
F 3 "" H 4450 3600 50  0000 C CNN
	1    4450 3600
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR04
U 1 1 57141DC9
P 4450 3800
F 0 "#PWR04" H 4450 3650 50  0001 C CNN
F 1 "+3.3V" H 4450 3940 50  0000 C CNN
F 2 "" H 4450 3800 50  0000 C CNN
F 3 "" H 4450 3800 50  0000 C CNN
	1    4450 3800
	0    -1   -1   0   
$EndComp
Text GLabel 2900 5200 2    60   Input ~ 0
PWM_38K
$Comp
L GND #PWR05
U 1 1 57142C4E
P 8000 1450
F 0 "#PWR05" H 8000 1200 50  0001 C CNN
F 1 "GND" H 8000 1300 50  0000 C CNN
F 2 "" H 8000 1450 50  0000 C CNN
F 3 "" H 8000 1450 50  0000 C CNN
	1    8000 1450
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR06
U 1 1 57142CA4
P 8000 1750
F 0 "#PWR06" H 8000 1600 50  0001 C CNN
F 1 "+3.3V" H 8000 1890 50  0000 C CNN
F 2 "" H 8000 1750 50  0000 C CNN
F 3 "" H 8000 1750 50  0000 C CNN
	1    8000 1750
	0    -1   -1   0   
$EndComp
$Comp
L NRF24L01-BRKOUT U1
U 1 1 5713E8BE
P 8850 1800
F 0 "U1" H 8450 2400 50  0000 L CNN
F 1 "NRF24L01-BRKOUT" H 8850 1200 50  0000 L CNN
F 2 "se8r01_smd:SE8R01_SMD" H 8850 1800 50  0001 L CNN
F 3 "" H 8850 1800 60  0000 C CNN
	1    8850 1800
	1    0    0    -1  
$EndComp
NoConn ~ 4550 4200
NoConn ~ 2800 3600
$Comp
L +3.3V #PWR07
U 1 1 57153B7B
P 2900 5500
F 0 "#PWR07" H 2900 5350 50  0001 C CNN
F 1 "+3.3V" H 2900 5640 50  0000 C CNN
F 2 "" H 2900 5500 50  0000 C CNN
F 3 "" H 2900 5500 50  0000 C CNN
	1    2900 5500
	0    1    1    0   
$EndComp
$Comp
L GND #PWR08
U 1 1 57153C2F
P 2900 5400
F 0 "#PWR08" H 2900 5150 50  0001 C CNN
F 1 "GND" H 2900 5250 50  0000 C CNN
F 2 "" H 2900 5400 50  0000 C CNN
F 3 "" H 2900 5400 50  0000 C CNN
	1    2900 5400
	0    -1   -1   0   
$EndComp
$Comp
L C C1
U 1 1 57177FF4
P 8000 1600
F 0 "C1" H 8025 1700 50  0000 L CNN
F 1 "C" H 8025 1500 50  0000 L CNN
F 2 "" H 8038 1450 50  0001 C CNN
F 3 "" H 8000 1600 50  0000 C CNN
	1    8000 1600
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG09
U 1 1 57142E4E
P 4050 3450
F 0 "#FLG09" H 4050 3545 50  0001 C CNN
F 1 "PWR_FLAG" H 4050 3630 50  0000 C CNN
F 2 "" H 4050 3450 50  0000 C CNN
F 3 "" H 4050 3450 50  0000 C CNN
	1    4050 3450
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG010
U 1 1 57142EAB
P 4050 3850
F 0 "#FLG010" H 4050 3945 50  0001 C CNN
F 1 "PWR_FLAG" H 4050 4030 50  0000 C CNN
F 2 "" H 4050 3850 50  0000 C CNN
F 3 "" H 4050 3850 50  0000 C CNN
	1    4050 3850
	1    0    0    -1  
$EndComp
Text Notes 8600 2800 0    60   ~ 0
// CE   A1\n// CSN  A0\n// IRQ  B0
$Comp
L 74HC595D U2
U 1 1 57630FAB
P 7350 4000
F 0 "U2" H 7129 4598 50  0000 L CNN
F 1 "74HC595D" H 7230 3015 50  0000 L CNN
F 2 "SOIC127P1030X265-16N:SOIC127P1030X265-16N" H 7350 4000 50  0001 L CNN
F 3 "" H 7350 4000 60  0000 C CNN
	1    7350 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 5763139E
P 6250 4400
F 0 "#PWR011" H 6250 4150 50  0001 C CNN
F 1 "GND" H 6250 4250 50  0000 C CNN
F 2 "" H 6250 4400 50  0000 C CNN
F 3 "" H 6250 4400 50  0000 C CNN
	1    6250 4400
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR012
U 1 1 576314B2
P 6250 3850
F 0 "#PWR012" H 6250 3700 50  0001 C CNN
F 1 "+3.3V" H 6250 3990 50  0000 C CNN
F 2 "" H 6250 3850 50  0000 C CNN
F 3 "" H 6250 3850 50  0000 C CNN
	1    6250 3850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4050 3450 4500 3450
Wire Wire Line
	4500 3450 4500 3700
Wire Wire Line
	4050 3950 4500 3950
Wire Wire Line
	4500 3950 4500 3800
Wire Wire Line
	4050 3850 4050 3950
Wire Wire Line
	8150 1750 8000 1750
Wire Wire Line
	8150 1700 8150 1750
Wire Wire Line
	8250 1700 8150 1700
Wire Wire Line
	8150 1450 8000 1450
Wire Wire Line
	8150 1500 8150 1450
Wire Wire Line
	8250 1500 8150 1500
Wire Wire Line
	4500 3700 4550 3700
Wire Wire Line
	2800 5400 2900 5400
Wire Wire Line
	2800 5500 2900 5500
Wire Wire Line
	8050 3700 8150 3700
Wire Wire Line
	8050 3900 8150 3900
Wire Wire Line
	8050 3800 8150 3800
Connection ~ 4500 3800
Connection ~ 4500 3600
Wire Wire Line
	2800 5200 2900 5200
Wire Wire Line
	4450 3600 4550 3600
Wire Wire Line
	4450 3800 4550 3800
Wire Wire Line
	2800 5100 2900 5100
Wire Wire Line
	8050 4400 8150 4400
Wire Wire Line
	8050 4300 8150 4300
Wire Wire Line
	8050 4200 8150 4200
Wire Wire Line
	8050 4100 8150 4100
Wire Wire Line
	8050 4000 8150 4000
Wire Wire Line
	4450 4600 4550 4600
Wire Wire Line
	4450 4500 4550 4500
Wire Wire Line
	4450 4400 4550 4400
Connection ~ 5650 2500
Wire Wire Line
	6900 2500 6900 2100
Connection ~ 5050 2500
Wire Wire Line
	5650 2500 5650 2100
Connection ~ 4450 2500
Wire Wire Line
	5050 2500 5050 2100
Connection ~ 3850 2500
Wire Wire Line
	4450 2500 4450 2100
Connection ~ 3250 2500
Wire Wire Line
	3850 2500 3850 2100
Connection ~ 2950 2500
Wire Wire Line
	3250 2100 3250 2500
Connection ~ 2050 2500
Wire Wire Line
	1450 2100 1450 2500
Connection ~ 2650 2500
Wire Wire Line
	2050 2100 2050 2500
Wire Wire Line
	1450 2500 6900 2500
Wire Wire Line
	2650 2100 2650 2500
Wire Wire Line
	2950 3100 2950 3000
Wire Wire Line
	2550 2800 2650 2800
Connection ~ 5650 1100
Wire Wire Line
	6500 1900 6600 1900
Wire Wire Line
	6900 1600 6900 1500
Wire Wire Line
	5250 1900 5350 1900
Wire Wire Line
	4650 1900 4750 1900
Wire Wire Line
	4050 1900 4150 1900
Wire Wire Line
	3450 1900 3550 1900
Wire Wire Line
	2850 1900 2950 1900
Wire Wire Line
	2250 1900 2350 1900
Wire Wire Line
	1650 1900 1750 1900
Wire Wire Line
	1050 1900 1150 1900
Connection ~ 5050 1100
Connection ~ 4450 1100
Connection ~ 3850 1100
Connection ~ 3250 1100
Connection ~ 2950 1100
Connection ~ 2050 1100
Connection ~ 2650 1100
Wire Wire Line
	1450 1100 6900 1100
Wire Wire Line
	5650 1600 5650 1500
Wire Wire Line
	5050 1600 5050 1500
Wire Wire Line
	4450 1600 4450 1500
Wire Wire Line
	3850 1600 3850 1500
Wire Wire Line
	3250 1600 3250 1500
Wire Wire Line
	2650 1600 2650 1500
Wire Wire Line
	2050 1600 2050 1500
Wire Wire Line
	2950 800  2950 750 
Wire Wire Line
	1450 1600 1450 1500
Wire Wire Line
	8250 1900 8150 1900
Wire Wire Line
	8250 2100 8150 2100
Wire Wire Line
	9450 2100 9550 2100
Wire Wire Line
	9450 1900 9550 1900
Wire Wire Line
	9450 1700 9550 1700
Wire Wire Line
	9450 1500 9550 1500
Wire Wire Line
	6650 3700 6250 3700
Wire Wire Line
	6250 3700 6250 3900
Wire Wire Line
	6250 3900 6650 3900
Connection ~ 6250 3850
Wire Wire Line
	6250 4500 6650 4500
Wire Wire Line
	6250 4200 6250 4500
Wire Wire Line
	6650 4200 6250 4200
Connection ~ 6250 4400
NoConn ~ 8050 4600
Text GLabel 6550 4000 0    60   Input ~ 0
SPI1_SCK
Text GLabel 6000 4300 0    60   Input ~ 0
SPI1_MOSI
Wire Wire Line
	6000 4300 6650 4300
Wire Wire Line
	6550 4000 6650 4000
Text GLabel 4450 4900 0    60   Input ~ 0
ShiftReg_Expose
Text GLabel 6550 4100 0    60   Input ~ 0
ShiftReg_Expose
Wire Wire Line
	6550 4100 6650 4100
NoConn ~ 2800 4900
NoConn ~ 2800 4800
NoConn ~ 2800 4700
NoConn ~ 2800 4600
NoConn ~ 2800 4500
NoConn ~ 2800 4400
NoConn ~ 2800 4300
Wire Wire Line
	4450 4300 4550 4300
Wire Wire Line
	4450 4700 4550 4700
Wire Wire Line
	4550 4800 4450 4800
NoConn ~ 4550 5300
NoConn ~ 4550 5200
NoConn ~ 4550 5100
NoConn ~ 3650 3050
Wire Wire Line
	4550 4900 4450 4900
NoConn ~ 2800 5000
$EndSCHEMATC
