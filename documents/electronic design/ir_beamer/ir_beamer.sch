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
NoConn ~ 6950 750 
NoConn ~ 6950 850 
NoConn ~ 6950 950 
NoConn ~ 6950 1050
NoConn ~ 6950 1150
NoConn ~ 6950 1550
NoConn ~ 6950 1650
NoConn ~ 6950 1950
NoConn ~ 6950 2050
NoConn ~ 6950 2150
NoConn ~ 6950 2250
NoConn ~ 6950 2350
NoConn ~ 7800 2650
NoConn ~ 7800 2550
NoConn ~ 7800 2450
NoConn ~ 7800 2350
NoConn ~ 7800 2150
NoConn ~ 7800 1250
NoConn ~ 7800 1150
NoConn ~ 7800 1050
$Comp
L LED D2
U 1 1 578759EC
P 1400 850
F 0 "D2" H 1400 950 50  0000 C CNN
F 1 "LED" H 1400 750 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 850 50  0001 C CNN
F 3 "" H 1400 850 50  0000 C CNN
	1    1400 850 
	-1   0    0    1   
$EndComp
Text Notes 8800 1650 0    60   ~ 0
A5     ------> SPI1_SCK\nA6     ------> SPI1_MISO\nA7     ------> SPI1_MOSI \n\nA0     ------> IR_PWM (TIM2 CH1 PWM out)\n\nA10 -> CE\nA11 -> CSN\nB0  -> IRQ\n
Text GLabel 7800 1750 2    60   Input ~ 0
SPI_SCK
Text GLabel 7800 1550 2    60   Input ~ 0
SPI_MOSI
Text GLabel 4750 1200 2    60   Input ~ 0
SPI_SCK
Text GLabel 4750 1000 2    60   Input ~ 0
SPI_MOSI
NoConn ~ 6950 1250
NoConn ~ 7800 1350
$Comp
L 2N7002K T1
U 1 1 578926A0
P 1900 950
F 0 "T1" V 1850 1100 50  0000 L CNN
F 1 "2N7002K" V 2150 850 50  0000 L CNN
F 2 "footprints:SOT23" H 2100 950 50  0001 L CNN
F 3 "" H 1900 950 60  0000 C CNN
	1    1900 950 
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR01
U 1 1 5787C121
P 3800 1500
F 0 "#PWR01" H 3800 1250 50  0001 C CNN
F 1 "GND" H 3800 1350 50  0000 C CNN
F 2 "" H 3800 1500 50  0000 C CNN
F 3 "" H 3800 1500 50  0000 C CNN
	1    3800 1500
	1    0    0    -1  
$EndComp
Text GLabel 1900 1150 2    60   Input ~ 0
IR_LED_1
Text GLabel 3800 800  0    60   Input ~ 0
IR_LED_1
Text GLabel 3800 900  0    60   Input ~ 0
IR_LED_2
Text GLabel 3800 1000 0    60   Input ~ 0
IR_LED_3
Text GLabel 3800 1100 0    60   Input ~ 0
IR_LED_4
Text GLabel 3800 1200 0    60   Input ~ 0
IR_LED_5
Text GLabel 3800 1300 0    60   Input ~ 0
IR_LED_6
Text GLabel 3800 1400 0    60   Input ~ 0
IR_LED_7
NoConn ~ 4750 900 
Text GLabel 7800 2250 2    60   Input ~ 0
IR_PWM
$Comp
L LED D1
U 1 1 57897D64
P 1400 1450
F 0 "D1" H 1400 1550 50  0000 C CNN
F 1 "LED" H 1400 1350 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 1450 50  0001 C CNN
F 3 "" H 1400 1450 50  0000 C CNN
	1    1400 1450
	-1   0    0    1   
$EndComp
$Comp
L 2N7002K T2
U 1 1 57897D6A
P 1900 1550
F 0 "T2" V 1850 1700 50  0000 L CNN
F 1 "2N7002K" V 2150 1450 50  0000 L CNN
F 2 "footprints:SOT23" H 2100 1550 50  0001 L CNN
F 3 "" H 1900 1550 60  0000 C CNN
	1    1900 1550
	0    -1   -1   0   
$EndComp
Text GLabel 1900 1750 2    60   Input ~ 0
IR_LED_2
$Comp
L LED D3
U 1 1 57897E48
P 1400 2050
F 0 "D3" H 1400 2150 50  0000 C CNN
F 1 "LED" H 1400 1950 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 2050 50  0001 C CNN
F 3 "" H 1400 2050 50  0000 C CNN
	1    1400 2050
	-1   0    0    1   
$EndComp
$Comp
L 2N7002K T3
U 1 1 57897E4E
P 1900 2150
F 0 "T3" V 1850 2300 50  0000 L CNN
F 1 "2N7002K" V 2150 2050 50  0000 L CNN
F 2 "footprints:SOT23" H 2100 2150 50  0001 L CNN
F 3 "" H 1900 2150 60  0000 C CNN
	1    1900 2150
	0    -1   -1   0   
$EndComp
Text GLabel 1900 2350 2    60   Input ~ 0
IR_LED_3
$Comp
L LED D4
U 1 1 57897E5C
P 1400 2650
F 0 "D4" H 1400 2750 50  0000 C CNN
F 1 "LED" H 1400 2550 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 2650 50  0001 C CNN
F 3 "" H 1400 2650 50  0000 C CNN
	1    1400 2650
	-1   0    0    1   
$EndComp
$Comp
L 2N7002K T4
U 1 1 57897E62
P 1900 2750
F 0 "T4" V 1850 2900 50  0000 L CNN
F 1 "2N7002K" V 2150 2650 50  0000 L CNN
F 2 "footprints:SOT23" H 2100 2750 50  0001 L CNN
F 3 "" H 1900 2750 60  0000 C CNN
	1    1900 2750
	0    -1   -1   0   
$EndComp
Text GLabel 1900 2950 2    60   Input ~ 0
IR_LED_4
$Comp
L LED D5
U 1 1 57898338
P 1400 3300
F 0 "D5" H 1400 3400 50  0000 C CNN
F 1 "LED" H 1400 3200 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 3300 50  0001 C CNN
F 3 "" H 1400 3300 50  0000 C CNN
	1    1400 3300
	-1   0    0    1   
$EndComp
$Comp
L 2N7002K T5
U 1 1 5789833E
P 1900 3400
F 0 "T5" V 1850 3550 50  0000 L CNN
F 1 "2N7002K" V 2150 3300 50  0000 L CNN
F 2 "footprints:SOT23" H 2100 3400 50  0001 L CNN
F 3 "" H 1900 3400 60  0000 C CNN
	1    1900 3400
	0    -1   -1   0   
$EndComp
Text GLabel 1900 3600 2    60   Input ~ 0
IR_LED_5
$Comp
L LED D6
U 1 1 5789834C
P 1400 3900
F 0 "D6" H 1400 4000 50  0000 C CNN
F 1 "LED" H 1400 3800 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 3900 50  0001 C CNN
F 3 "" H 1400 3900 50  0000 C CNN
	1    1400 3900
	-1   0    0    1   
$EndComp
$Comp
L 2N7002K T6
U 1 1 57898352
P 1900 4000
F 0 "T6" V 1850 4150 50  0000 L CNN
F 1 "2N7002K" V 2150 3900 50  0000 L CNN
F 2 "footprints:SOT23" H 2100 4000 50  0001 L CNN
F 3 "" H 1900 4000 60  0000 C CNN
	1    1900 4000
	0    -1   -1   0   
$EndComp
Text GLabel 1900 4200 2    60   Input ~ 0
IR_LED_6
$Comp
L LED D7
U 1 1 57898360
P 1400 4500
F 0 "D7" H 1400 4600 50  0000 C CNN
F 1 "LED" H 1400 4400 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 4500 50  0001 C CNN
F 3 "" H 1400 4500 50  0000 C CNN
	1    1400 4500
	-1   0    0    1   
$EndComp
$Comp
L 2N7002K T7
U 1 1 57898366
P 1900 4600
F 0 "T7" V 1850 4750 50  0000 L CNN
F 1 "2N7002K" V 2150 4500 50  0000 L CNN
F 2 "footprints:SOT23" H 2100 4600 50  0001 L CNN
F 3 "" H 1900 4600 60  0000 C CNN
	1    1900 4600
	0    -1   -1   0   
$EndComp
Text GLabel 1900 4800 2    60   Input ~ 0
IR_LED_7
$Comp
L LED D8
U 1 1 578C79D4
P 1400 5150
F 0 "D8" H 1400 5250 50  0000 C CNN
F 1 "LED" H 1400 5050 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 5150 50  0001 C CNN
F 3 "" H 1400 5150 50  0000 C CNN
	1    1400 5150
	-1   0    0    1   
$EndComp
$Comp
L 2N7002K T8
U 1 1 578C79DA
P 1900 5250
F 0 "T8" V 1850 5400 50  0000 L CNN
F 1 "2N7002K" V 2150 5150 50  0000 L CNN
F 2 "footprints:SOT23" H 2100 5250 50  0001 L CNN
F 3 "" H 1900 5250 60  0000 C CNN
	1    1900 5250
	0    -1   -1   0   
$EndComp
Text GLabel 1900 5450 2    60   Input ~ 0
IR_LED_8
$Comp
L LED D9
U 1 1 578C79E8
P 1400 5750
F 0 "D9" H 1400 5850 50  0000 C CNN
F 1 "LED" H 1400 5650 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 5750 50  0001 C CNN
F 3 "" H 1400 5750 50  0000 C CNN
	1    1400 5750
	-1   0    0    1   
$EndComp
$Comp
L 2N7002K T9
U 1 1 578C79EE
P 1900 5850
F 0 "T9" V 1850 6000 50  0000 L CNN
F 1 "2N7002K" V 2150 5750 50  0000 L CNN
F 2 "footprints:SOT23" H 2100 5850 50  0001 L CNN
F 3 "" H 1900 5850 60  0000 C CNN
	1    1900 5850
	0    -1   -1   0   
$EndComp
Text GLabel 1900 6050 2    60   Input ~ 0
IR_LED_9
$Comp
L LED D10
U 1 1 578C79FC
P 1400 6350
F 0 "D10" H 1400 6450 50  0000 C CNN
F 1 "LED" H 1400 6250 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 6350 50  0001 C CNN
F 3 "" H 1400 6350 50  0000 C CNN
	1    1400 6350
	-1   0    0    1   
$EndComp
$Comp
L 2N7002K T10
U 1 1 578C7A02
P 1900 6450
F 0 "T10" V 1850 6600 50  0000 L CNN
F 1 "2N7002K" V 2150 6350 50  0000 L CNN
F 2 "footprints:SOT23" H 2100 6450 50  0001 L CNN
F 3 "" H 1900 6450 60  0000 C CNN
	1    1900 6450
	0    -1   -1   0   
$EndComp
Text GLabel 1900 6650 2    60   Input ~ 0
IR_LED_10
$Comp
L GND #PWR02
U 1 1 578C7A0A
P 4750 4700
F 0 "#PWR02" H 4750 4450 50  0001 C CNN
F 1 "GND" H 4750 4550 50  0000 C CNN
F 2 "" H 4750 4700 50  0000 C CNN
F 3 "" H 4750 4700 50  0000 C CNN
	1    4750 4700
	0    -1   -1   0   
$EndComp
$Comp
L LED D11
U 1 1 578C7E10
P 1400 7000
F 0 "D11" H 1400 7100 50  0000 C CNN
F 1 "LED" H 1400 6900 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 7000 50  0001 C CNN
F 3 "" H 1400 7000 50  0000 C CNN
	1    1400 7000
	-1   0    0    1   
$EndComp
$Comp
L 2N7002K T11
U 1 1 578C7E16
P 1900 7100
F 0 "T11" V 1850 7250 50  0000 L CNN
F 1 "2N7002K" V 2150 7000 50  0000 L CNN
F 2 "footprints:SOT23" H 2100 7100 50  0001 L CNN
F 3 "" H 1900 7100 60  0000 C CNN
	1    1900 7100
	0    -1   -1   0   
$EndComp
Text GLabel 1900 7300 2    60   Input ~ 0
IR_LED_DATA
Text GLabel 4750 2400 2    60   Input ~ 0
SPI_SCK
$Comp
L +3.3V #PWR03
U 1 1 578C8F52
P 4750 2000
F 0 "#PWR03" H 4750 1850 50  0001 C CNN
F 1 "+3.3V" H 4750 2140 50  0000 C CNN
F 2 "" H 4750 2000 50  0000 C CNN
F 3 "" H 4750 2000 50  0000 C CNN
	1    4750 2000
	0    1    1    0   
$EndComp
$Comp
L GND #PWR04
U 1 1 578C8F58
P 3800 2700
F 0 "#PWR04" H 3800 2450 50  0001 C CNN
F 1 "GND" H 3800 2550 50  0000 C CNN
F 2 "" H 3800 2700 50  0000 C CNN
F 3 "" H 3800 2700 50  0000 C CNN
	1    3800 2700
	1    0    0    -1  
$EndComp
Text GLabel 3800 2000 0    60   Input ~ 0
IR_LED_8
Text GLabel 3800 2100 0    60   Input ~ 0
IR_LED_9
Text GLabel 3800 2200 0    60   Input ~ 0
IR_LED_10
Text GLabel 3800 2600 0    60   Input ~ 0
IR_LED_DATA
NoConn ~ 4750 2100
NoConn ~ 4750 2700
Text GLabel 6500 4300 0    60   Input ~ 0
Latch
NoConn ~ 3800 2300
NoConn ~ 3800 2400
NoConn ~ 3800 2500
$Comp
L 74HC595D U1
U 1 1 578CC6D2
P 4250 1550
F 0 "U1" H 4250 2400 60  0000 C CNN
F 1 "74HC595D" H 4300 1500 60  0000 C CNN
F 2 "footprints:74HC595D_SO16" H 4000 1550 60  0001 C CNN
F 3 "" H 4000 1550 60  0000 C CNN
	1    4250 1550
	1    0    0    -1  
$EndComp
$Comp
L 74HC595D U3
U 1 1 578CC74D
P 4250 2750
F 0 "U3" H 4250 3600 60  0000 C CNN
F 1 "74HC595D" H 4300 2700 60  0000 C CNN
F 2 "footprints:74HC595D_SO16" H 4000 2750 60  0001 C CNN
F 3 "" H 4000 2750 60  0000 C CNN
	1    4250 2750
	1    0    0    -1  
$EndComp
NoConn ~ 7800 2050
Text GLabel 7800 1850 2    60   Input ~ 0
Latch
Text GLabel 6600 3650 0    60   Input ~ 0
IR_PWM
$Comp
L +3V3 #PWR05
U 1 1 578F2608
P 6150 2650
F 0 "#PWR05" H 6150 2500 50  0001 C CNN
F 1 "+3V3" H 6150 2790 50  0000 C CNN
F 2 "" H 6150 2650 50  0000 C CNN
F 3 "" H 6150 2650 50  0000 C CNN
	1    6150 2650
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG06
U 1 1 578F2694
P 6400 2550
F 0 "#FLG06" H 6400 2645 50  0001 C CNN
F 1 "PWR_FLAG" H 6400 2730 50  0000 C CNN
F 2 "" H 6400 2550 50  0000 C CNN
F 3 "" H 6400 2550 50  0000 C CNN
	1    6400 2550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5787C14A
P 6400 2550
F 0 "#PWR07" H 6400 2300 50  0001 C CNN
F 1 "GND" H 6400 2400 50  0000 C CNN
F 2 "" H 6400 2550 50  0000 C CNN
F 3 "" H 6400 2550 50  0000 C CNN
	1    6400 2550
	0    1    1    0   
$EndComp
$Comp
L PWR_FLAG #FLG08
U 1 1 578F289B
P 6150 2650
F 0 "#FLG08" H 6150 2745 50  0001 C CNN
F 1 "PWR_FLAG" H 6150 2830 50  0000 C CNN
F 2 "" H 6150 2650 50  0000 C CNN
F 3 "" H 6150 2650 50  0000 C CNN
	1    6150 2650
	-1   0    0    1   
$EndComp
$Comp
L STM32F103HEADER U2
U 1 1 578F444B
P 7200 1800
F 0 "U2" H 7350 3050 60  0000 C CNN
F 1 "STM32F103HEADER" H 7400 750 60  0000 C CNN
F 2 "footprints:STM32F103_header_footprint" H 6550 2800 60  0001 C CNN
F 3 "" H 6550 2800 60  0000 C CNN
	1    7200 1800
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR09
U 1 1 578F5D3D
P 8100 3650
F 0 "#PWR09" H 8100 3500 50  0001 C CNN
F 1 "+3V3" H 8100 3790 50  0000 C CNN
F 2 "" H 8100 3650 50  0000 C CNN
F 3 "" H 8100 3650 50  0000 C CNN
	1    8100 3650
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR010
U 1 1 578F61E2
P 4750 800
F 0 "#PWR010" H 4750 650 50  0001 C CNN
F 1 "+3V3" H 4750 940 50  0000 C CNN
F 2 "" H 4750 800 50  0000 C CNN
F 3 "" H 4750 800 50  0000 C CNN
	1    4750 800 
	0    1    1    0   
$EndComp
$Comp
L SE8R01_module U4
U 1 1 578FCE1F
P 4450 3800
F 0 "U4" H 4250 4250 50  0000 L CNN
F 1 "SE8R01_module" H 4000 3500 50  0000 L CNN
F 2 "footprints:SE8R01_SMD" H 4150 4150 50  0001 L CNN
F 3 "" H 4500 3800 60  0000 C CNN
	1    4450 3800
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR011
U 1 1 578FD7B5
P 3650 3400
F 0 "#PWR011" H 3650 3250 50  0001 C CNN
F 1 "+3V3" H 3650 3540 50  0000 C CNN
F 2 "" H 3650 3400 50  0000 C CNN
F 3 "" H 3650 3400 50  0000 C CNN
	1    3650 3400
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR012
U 1 1 578FDB5C
P 3350 3700
F 0 "#PWR012" H 3350 3450 50  0001 C CNN
F 1 "GND" H 3350 3550 50  0000 C CNN
F 2 "" H 3350 3700 50  0000 C CNN
F 3 "" H 3350 3700 50  0000 C CNN
	1    3350 3700
	0    1    1    0   
$EndComp
$Comp
L C C1
U 1 1 578FDDE6
P 3650 3550
F 0 "C1" H 3675 3650 50  0000 L CNN
F 1 "100uf" H 3675 3450 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3688 3400 50  0001 C CNN
F 3 "" H 3650 3550 50  0000 C CNN
	1    3650 3550
	1    0    0    -1  
$EndComp
Text GLabel 3850 3850 0    60   Input ~ 0
SE8R01_CE
Text GLabel 3850 4000 0    60   Input ~ 0
SE8R01_CSN
Text GLabel 4700 3550 2    60   Input ~ 0
SPI_SCK
Text GLabel 6650 3850 0    60   Input ~ 0
SPI_MOSI
Text GLabel 4700 3850 2    60   Input ~ 0
SPI_MISO
Text GLabel 7800 1650 2    60   Input ~ 0
SPI_MISO
Text GLabel 4700 4000 2    60   Input ~ 0
SE8R01_IRQ
Text GLabel 7800 1450 2    60   Input ~ 0
SE8R01_IRQ
Text GLabel 6950 1350 0    60   Input ~ 0
SE8R01_CE
Text GLabel 6950 1450 0    60   Input ~ 0
SE8R01_CSN
NoConn ~ 7800 1950
$Comp
L Via P1
U 1 1 5790367E
P 6850 3850
F 0 "P1" H 6850 3950 50  0000 C CNN
F 1 "Via" V 6950 3850 50  0000 C CNN
F 2 "footprints:Via" H 6850 3850 50  0001 C CNN
F 3 "" H 6850 3850 50  0000 C CNN
	1    6850 3850
	1    0    0    -1  
$EndComp
$Comp
L Via P2
U 1 1 579042CF
P 6700 4100
F 0 "P2" H 6700 4200 50  0000 C CNN
F 1 "Via" V 6800 4100 50  0000 C CNN
F 2 "footprints:Via" H 6700 4100 50  0001 C CNN
F 3 "" H 6700 4100 50  0000 C CNN
	1    6700 4100
	0    -1   -1   0   
$EndComp
$Comp
L Via P5
U 1 1 57904E64
P 8300 3650
F 0 "P5" H 8300 3750 50  0000 C CNN
F 1 "Via" V 8400 3650 50  0000 C CNN
F 2 "footprints:Via" H 8300 3650 50  0001 C CNN
F 3 "" H 8300 3650 50  0000 C CNN
	1    8300 3650
	1    0    0    -1  
$EndComp
$Comp
L Via P3
U 1 1 57905310
P 6800 3650
F 0 "P3" H 6800 3750 50  0000 C CNN
F 1 "Via" V 6900 3650 50  0000 C CNN
F 2 "footprints:Via" H 6800 3650 50  0001 C CNN
F 3 "" H 6800 3650 50  0000 C CNN
	1    6800 3650
	1    0    0    -1  
$EndComp
$Comp
L Via P4
U 1 1 5790636F
P 3350 3500
F 0 "P4" H 3350 3600 50  0000 C CNN
F 1 "Via" V 3450 3500 50  0000 C CNN
F 2 "footprints:Via" H 3350 3500 50  0001 C CNN
F 3 "" H 3350 3500 50  0000 C CNN
	1    3350 3500
	0    -1   -1   0   
$EndComp
$Comp
L Via P7
U 1 1 57907AF1
P 6900 4550
F 0 "P7" H 6900 4650 50  0000 C CNN
F 1 "Via" V 7000 4550 50  0000 C CNN
F 2 "footprints:Via" H 6900 4550 50  0001 C CNN
F 3 "" H 6900 4550 50  0000 C CNN
	1    6900 4550
	0    -1   -1   0   
$EndComp
$Comp
L Via P8
U 1 1 579081D8
P 6700 4550
F 0 "P8" H 6700 4650 50  0000 C CNN
F 1 "Via" V 6800 4550 50  0000 C CNN
F 2 "footprints:Via" H 6700 4550 50  0001 C CNN
F 3 "" H 6700 4550 50  0000 C CNN
	1    6700 4550
	0    -1   -1   0   
$EndComp
Text GLabel 4750 2500 2    60   Input ~ 0
Latch
Text GLabel 4700 3700 2    60   Input ~ 0
SPI_MOSI
Text GLabel 4750 1300 2    60   Input ~ 0
Latch
$Comp
L Via P9
U 1 1 5790E00A
P 8300 3900
F 0 "P9" H 8300 4000 50  0000 C CNN
F 1 "Via" V 8400 3900 50  0000 C CNN
F 2 "footprints:Via" H 8300 3900 50  0001 C CNN
F 3 "" H 8300 3900 50  0000 C CNN
	1    8300 3900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 5790E39C
P 7750 4700
F 0 "#PWR013" H 7750 4450 50  0001 C CNN
F 1 "GND" H 7750 4550 50  0000 C CNN
F 2 "" H 7750 4700 50  0000 C CNN
F 3 "" H 7750 4700 50  0000 C CNN
	1    7750 4700
	0    1    1    0   
$EndComp
$Comp
L Via P10
U 1 1 5790E41F
P 7850 4500
F 0 "P10" H 7850 4600 50  0000 C CNN
F 1 "Via" V 7950 4500 50  0000 C CNN
F 2 "footprints:Via" H 7850 4500 50  0001 C CNN
F 3 "" H 7850 4500 50  0000 C CNN
	1    7850 4500
	0    -1   -1   0   
$EndComp
Text GLabel 3950 4700 0    60   Input ~ 0
PWM_GND
$Comp
L +5V #PWR014
U 1 1 57910C6F
P 6700 2450
F 0 "#PWR014" H 6700 2300 50  0001 C CNN
F 1 "+5V" H 6700 2590 50  0000 C CNN
F 2 "" H 6700 2450 50  0000 C CNN
F 3 "" H 6700 2450 50  0000 C CNN
	1    6700 2450
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG015
U 1 1 57910CFC
P 6700 2450
F 0 "#FLG015" H 6700 2545 50  0001 C CNN
F 1 "PWR_FLAG" H 6700 2630 50  0000 C CNN
F 2 "" H 6700 2450 50  0000 C CNN
F 3 "" H 6700 2450 50  0000 C CNN
	1    6700 2450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR016
U 1 1 57911181
P 1200 6350
F 0 "#PWR016" H 1200 6200 50  0001 C CNN
F 1 "+5V" H 1200 6490 50  0000 C CNN
F 2 "" H 1200 6350 50  0000 C CNN
F 3 "" H 1200 6350 50  0000 C CNN
	1    1200 6350
	0    -1   -1   0   
$EndComp
$Comp
L R R5
U 1 1 57911702
P 4100 4700
F 0 "R5" V 4180 4700 50  0000 C CNN
F 1 "100" V 4100 4700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4030 4700 50  0001 C CNN
F 3 "" H 4100 4700 50  0000 C CNN
	1    4100 4700
	0    -1   -1   0   
$EndComp
$Comp
L 2N7002K T12
U 1 1 5790F5F3
P 4550 4800
F 0 "T12" V 4500 4950 50  0000 L CNN
F 1 "2N7002K" V 4800 4700 50  0000 L CNN
F 2 "footprints:SOT23" H 4750 4800 50  0001 L CNN
F 3 "" H 4550 4800 60  0000 C CNN
	1    4550 4800
	0    -1   -1   0   
$EndComp
Text GLabel 4550 5000 2    60   Input ~ 0
IR_PWM
Text GLabel 2100 6350 2    60   Input ~ 0
PWM_GND
Text GLabel 2100 5750 2    60   Input ~ 0
PWM_GND
Text GLabel 2100 5150 2    60   Input ~ 0
PWM_GND
Text GLabel 2100 4500 2    60   Input ~ 0
PWM_GND
Text GLabel 2100 3900 2    60   Input ~ 0
PWM_GND
Text GLabel 2100 3300 2    60   Input ~ 0
PWM_GND
Text GLabel 2100 2650 2    60   Input ~ 0
PWM_GND
Text GLabel 2100 2050 2    60   Input ~ 0
PWM_GND
Text GLabel 2100 1450 2    60   Input ~ 0
PWM_GND
Text GLabel 2100 850  2    60   Input ~ 0
PWM_GND
$Comp
L +5V #PWR017
U 1 1 5791B923
P 1200 5750
F 0 "#PWR017" H 1200 5600 50  0001 C CNN
F 1 "+5V" H 1200 5890 50  0000 C CNN
F 2 "" H 1200 5750 50  0000 C CNN
F 3 "" H 1200 5750 50  0000 C CNN
	1    1200 5750
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR018
U 1 1 5791B9AC
P 1200 5150
F 0 "#PWR018" H 1200 5000 50  0001 C CNN
F 1 "+5V" H 1200 5290 50  0000 C CNN
F 2 "" H 1200 5150 50  0000 C CNN
F 3 "" H 1200 5150 50  0000 C CNN
	1    1200 5150
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR019
U 1 1 5791BB5B
P 1200 4500
F 0 "#PWR019" H 1200 4350 50  0001 C CNN
F 1 "+5V" H 1200 4640 50  0000 C CNN
F 2 "" H 1200 4500 50  0000 C CNN
F 3 "" H 1200 4500 50  0000 C CNN
	1    1200 4500
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR020
U 1 1 5791BBE4
P 1200 3900
F 0 "#PWR020" H 1200 3750 50  0001 C CNN
F 1 "+5V" H 1200 4040 50  0000 C CNN
F 2 "" H 1200 3900 50  0000 C CNN
F 3 "" H 1200 3900 50  0000 C CNN
	1    1200 3900
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR021
U 1 1 5791BC6D
P 1200 3300
F 0 "#PWR021" H 1200 3150 50  0001 C CNN
F 1 "+5V" H 1200 3440 50  0000 C CNN
F 2 "" H 1200 3300 50  0000 C CNN
F 3 "" H 1200 3300 50  0000 C CNN
	1    1200 3300
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR022
U 1 1 5791BF18
P 1200 2650
F 0 "#PWR022" H 1200 2500 50  0001 C CNN
F 1 "+5V" H 1200 2790 50  0000 C CNN
F 2 "" H 1200 2650 50  0000 C CNN
F 3 "" H 1200 2650 50  0000 C CNN
	1    1200 2650
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR023
U 1 1 5791BFA1
P 1200 2050
F 0 "#PWR023" H 1200 1900 50  0001 C CNN
F 1 "+5V" H 1200 2190 50  0000 C CNN
F 2 "" H 1200 2050 50  0000 C CNN
F 3 "" H 1200 2050 50  0000 C CNN
	1    1200 2050
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR024
U 1 1 5791C02A
P 1200 1450
F 0 "#PWR024" H 1200 1300 50  0001 C CNN
F 1 "+5V" H 1200 1590 50  0000 C CNN
F 2 "" H 1200 1450 50  0000 C CNN
F 3 "" H 1200 1450 50  0000 C CNN
	1    1200 1450
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR025
U 1 1 5791C22D
P 1200 850
F 0 "#PWR025" H 1200 700 50  0001 C CNN
F 1 "+5V" H 1200 990 50  0000 C CNN
F 2 "" H 1200 850 50  0000 C CNN
F 3 "" H 1200 850 50  0000 C CNN
	1    1200 850 
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR026
U 1 1 5791CFE3
P 1200 7000
F 0 "#PWR026" H 1200 6850 50  0001 C CNN
F 1 "+5V" H 1200 7140 50  0000 C CNN
F 2 "" H 1200 7000 50  0000 C CNN
F 3 "" H 1200 7000 50  0000 C CNN
	1    1200 7000
	0    -1   -1   0   
$EndComp
Text GLabel 2100 7000 2    60   Input ~ 0
PWM_GND
Text GLabel 4750 1500 2    60   Input ~ 0
Cascade
Text GLabel 4750 2200 2    60   Input ~ 0
Cascade
Text GLabel 6550 4750 0    60   Input ~ 0
Cascade
Text GLabel 4750 1400 2    60   Input ~ 0
MR_NOT
Text GLabel 4750 2600 2    60   Input ~ 0
MR_NOT
Text GLabel 4750 2300 2    60   Input ~ 0
OE_NOT
Text GLabel 4750 1100 2    60   Input ~ 0
OE_NOT
Text GLabel 6950 1750 0    60   Input ~ 0
OE_NOT
Text GLabel 6950 1850 0    60   Input ~ 0
MR_NOT
Text GLabel 6550 5150 0    60   Input ~ 0
OE_NOT
Text GLabel 6600 5550 0    60   Input ~ 0
MR_NOT
$Comp
L Via P6
U 1 1 57964292
P 6700 4950
F 0 "P6" H 6700 5050 50  0000 C CNN
F 1 "Via" V 6800 4950 50  0000 C CNN
F 2 "footprints:Via" H 6700 4950 50  0001 C CNN
F 3 "" H 6700 4950 50  0000 C CNN
	1    6700 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P11
U 1 1 57964337
P 6700 5350
F 0 "P11" H 6700 5450 50  0000 C CNN
F 1 "Via" V 6800 5350 50  0000 C CNN
F 2 "footprints:Via" H 6700 5350 50  0001 C CNN
F 3 "" H 6700 5350 50  0000 C CNN
	1    6700 5350
	0    -1   -1   0   
$EndComp
$Comp
L Via P12
U 1 1 57964AC1
P 6900 4950
F 0 "P12" H 6900 5050 50  0000 C CNN
F 1 "Via" V 7000 4950 50  0000 C CNN
F 2 "footprints:Via" H 6900 4950 50  0001 C CNN
F 3 "" H 6900 4950 50  0000 C CNN
	1    6900 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P13
U 1 1 57964C19
P 6900 5350
F 0 "P13" H 6900 5450 50  0000 C CNN
F 1 "Via" V 7000 5350 50  0000 C CNN
F 2 "footprints:Via" H 6900 5350 50  0001 C CNN
F 3 "" H 6900 5350 50  0000 C CNN
	1    6900 5350
	0    -1   -1   0   
$EndComp
$Comp
L Via P14
U 1 1 5796661B
P 6900 4100
F 0 "P14" H 6900 4200 50  0000 C CNN
F 1 "Via" V 7000 4100 50  0000 C CNN
F 2 "footprints:Via" H 6900 4100 50  0001 C CNN
F 3 "" H 6900 4100 50  0000 C CNN
	1    6900 4100
	0    -1   -1   0   
$EndComp
$Comp
L Via P15
U 1 1 57967F51
P 7100 4950
F 0 "P15" H 7100 5050 50  0000 C CNN
F 1 "Via" V 7200 4950 50  0000 C CNN
F 2 "footprints:Via" H 7100 4950 50  0001 C CNN
F 3 "" H 7100 4950 50  0000 C CNN
	1    7100 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P16
U 1 1 579685FA
P 8050 4500
F 0 "P16" H 8050 4600 50  0000 C CNN
F 1 "Via" V 8150 4500 50  0000 C CNN
F 2 "footprints:Via" H 8050 4500 50  0001 C CNN
F 3 "" H 8050 4500 50  0000 C CNN
	1    8050 4500
	0    -1   -1   0   
$EndComp
$Comp
L Via P17
U 1 1 579689E2
P 7600 3700
F 0 "P17" H 7600 3800 50  0000 C CNN
F 1 "Via" V 7700 3700 50  0000 C CNN
F 2 "footprints:Via" H 7600 3700 50  0001 C CNN
F 3 "" H 7600 3700 50  0000 C CNN
	1    7600 3700
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR027
U 1 1 579691C2
P 7600 3900
F 0 "#PWR027" H 7600 3750 50  0001 C CNN
F 1 "+5V" H 7600 4040 50  0000 C CNN
F 2 "" H 7600 3900 50  0000 C CNN
F 3 "" H 7600 3900 50  0000 C CNN
	1    7600 3900
	0    -1   -1   0   
$EndComp
$Comp
L Via P18
U 1 1 5798A9D4
P 8250 4950
F 0 "P18" H 8250 5050 50  0000 C CNN
F 1 "Via" V 8350 4950 50  0000 C CNN
F 2 "footprints:Via" H 8250 4950 50  0001 C CNN
F 3 "" H 8250 4950 50  0000 C CNN
	1    8250 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P19
U 1 1 5798AA73
P 8400 4950
F 0 "P19" H 8400 5050 50  0000 C CNN
F 1 "Via" V 8500 4950 50  0000 C CNN
F 2 "footprints:Via" H 8400 4950 50  0001 C CNN
F 3 "" H 8400 4950 50  0000 C CNN
	1    8400 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P20
U 1 1 5798AB15
P 8550 4950
F 0 "P20" H 8550 5050 50  0000 C CNN
F 1 "Via" V 8650 4950 50  0000 C CNN
F 2 "footprints:Via" H 8550 4950 50  0001 C CNN
F 3 "" H 8550 4950 50  0000 C CNN
	1    8550 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P21
U 1 1 5798ACCF
P 8700 4950
F 0 "P21" H 8700 5050 50  0000 C CNN
F 1 "Via" V 8800 4950 50  0000 C CNN
F 2 "footprints:Via" H 8700 4950 50  0001 C CNN
F 3 "" H 8700 4950 50  0000 C CNN
	1    8700 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P22
U 1 1 5798ACD5
P 8850 4950
F 0 "P22" H 8850 5050 50  0000 C CNN
F 1 "Via" V 8950 4950 50  0000 C CNN
F 2 "footprints:Via" H 8850 4950 50  0001 C CNN
F 3 "" H 8850 4950 50  0000 C CNN
	1    8850 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P23
U 1 1 5798ACDB
P 9000 4950
F 0 "P23" H 9000 5050 50  0000 C CNN
F 1 "Via" V 9100 4950 50  0000 C CNN
F 2 "footprints:Via" H 9000 4950 50  0001 C CNN
F 3 "" H 9000 4950 50  0000 C CNN
	1    9000 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P24
U 1 1 5798ACE1
P 9150 4950
F 0 "P24" H 9150 5050 50  0000 C CNN
F 1 "Via" V 9250 4950 50  0000 C CNN
F 2 "footprints:Via" H 9150 4950 50  0001 C CNN
F 3 "" H 9150 4950 50  0000 C CNN
	1    9150 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P25
U 1 1 5798AE44
P 9300 4950
F 0 "P25" H 9300 5050 50  0000 C CNN
F 1 "Via" V 9400 4950 50  0000 C CNN
F 2 "footprints:Via" H 9300 4950 50  0001 C CNN
F 3 "" H 9300 4950 50  0000 C CNN
	1    9300 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P26
U 1 1 5798AE4A
P 9450 4950
F 0 "P26" H 9450 5050 50  0000 C CNN
F 1 "Via" V 9550 4950 50  0000 C CNN
F 2 "footprints:Via" H 9450 4950 50  0001 C CNN
F 3 "" H 9450 4950 50  0000 C CNN
	1    9450 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P27
U 1 1 5798AE50
P 9600 4950
F 0 "P27" H 9600 5050 50  0000 C CNN
F 1 "Via" V 9700 4950 50  0000 C CNN
F 2 "footprints:Via" H 9600 4950 50  0001 C CNN
F 3 "" H 9600 4950 50  0000 C CNN
	1    9600 4950
	0    -1   -1   0   
$EndComp
$Comp
L Via P28
U 1 1 5798AE56
P 9750 4950
F 0 "P28" H 9750 5050 50  0000 C CNN
F 1 "Via" V 9850 4950 50  0000 C CNN
F 2 "footprints:Via" H 9750 4950 50  0001 C CNN
F 3 "" H 9750 4950 50  0000 C CNN
	1    9750 4950
	0    -1   -1   0   
$EndComp
Text GLabel 8100 5150 0    60   Input ~ 0
PWM_GND
Wire Wire Line
	7750 4700 8050 4700
Connection ~ 8050 4700
Connection ~ 7850 4700
Connection ~ 6900 5150
Connection ~ 6700 4300
Wire Wire Line
	6500 4300 6900 4300
Connection ~ 6700 5150
Wire Wire Line
	6550 5150 7100 5150
Connection ~ 6700 5550
Wire Wire Line
	6600 5550 6900 5550
Connection ~ 6700 4750
Wire Wire Line
	6550 4750 6900 4750
Wire Wire Line
	6950 2450 6700 2450
Wire Wire Line
	6400 2550 6950 2550
Wire Wire Line
	3350 3700 3850 3700
Wire Wire Line
	3850 3400 3850 3550
Wire Wire Line
	6950 2650 6150 2650
Connection ~ 3650 3700
Wire Wire Line
	3650 3400 3850 3400
Wire Wire Line
	8100 3900 8100 3650
Wire Wire Line
	8100 5150 9750 5150
Connection ~ 8250 5150
Connection ~ 8400 5150
Connection ~ 8550 5150
Connection ~ 8700 5150
Connection ~ 8850 5150
Connection ~ 9000 5150
Connection ~ 9150 5150
Connection ~ 9300 5150
Connection ~ 9450 5150
Connection ~ 9600 5150
$EndSCHEMATC
