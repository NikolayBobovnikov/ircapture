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
P 1400 1150
F 0 "D2" H 1400 1250 50  0000 C CNN
F 1 "LED" H 1400 1050 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 1150 50  0001 C CNN
F 3 "" H 1400 1150 50  0000 C CNN
	1    1400 1150
	-1   0    0    1   
$EndComp
Text Notes 8800 1650 0    60   ~ 0
A5     ------> SPI1_SCK\nA6     ------> SPI1_MISO\nA7     ------> SPI1_MOSI \n\nA0     ------> IR_PWM (TIM2 CH1 PWM out)\n\nA10 -> CE\nA11 -> CSN\nB0  -> IRQ\n
Text GLabel 7800 1750 2    60   Input ~ 0
SPI_SCK
Text GLabel 7800 1550 2    60   Input ~ 0
SPI_MOSI
Text GLabel 4700 1050 2    60   Input ~ 0
SPI_SCK
Text GLabel 4700 850  2    60   Input ~ 0
SPI_MOSI
NoConn ~ 6950 1250
NoConn ~ 7800 1350
Text GLabel 3800 2400 0    60   Input ~ 0
IR_LED_1
Text GLabel 3800 850  0    60   Input ~ 0
IR_LED_2
Text GLabel 3800 950  0    60   Input ~ 0
IR_LED_3
Text GLabel 3800 1050 0    60   Input ~ 0
IR_LED_4
Text GLabel 3800 1150 0    60   Input ~ 0
IR_LED_5
Text GLabel 3800 1250 0    60   Input ~ 0
IR_LED_6
Text GLabel 3800 1350 0    60   Input ~ 0
IR_LED_7
Text GLabel 7800 2250 2    60   Input ~ 0
IR_PWM
$Comp
L LED D1
U 1 1 57897D64
P 1400 1750
F 0 "D1" H 1400 1850 50  0000 C CNN
F 1 "LED" H 1400 1650 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 1750 50  0001 C CNN
F 3 "" H 1400 1750 50  0000 C CNN
	1    1400 1750
	-1   0    0    1   
$EndComp
Text GLabel 1600 1750 2    60   Input ~ 0
IR_LED_2
$Comp
L LED D3
U 1 1 57897E48
P 1400 2350
F 0 "D3" H 1400 2450 50  0000 C CNN
F 1 "LED" H 1400 2250 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 2350 50  0001 C CNN
F 3 "" H 1400 2350 50  0000 C CNN
	1    1400 2350
	-1   0    0    1   
$EndComp
Text GLabel 1600 2350 2    60   Input ~ 0
IR_LED_3
$Comp
L LED D4
U 1 1 57897E5C
P 1400 2950
F 0 "D4" H 1400 3050 50  0000 C CNN
F 1 "LED" H 1400 2850 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 2950 50  0001 C CNN
F 3 "" H 1400 2950 50  0000 C CNN
	1    1400 2950
	-1   0    0    1   
$EndComp
Text GLabel 1600 2950 2    60   Input ~ 0
IR_LED_4
$Comp
L LED D5
U 1 1 57898338
P 1400 3550
F 0 "D5" H 1400 3650 50  0000 C CNN
F 1 "LED" H 1400 3450 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 3550 50  0001 C CNN
F 3 "" H 1400 3550 50  0000 C CNN
	1    1400 3550
	-1   0    0    1   
$EndComp
Text GLabel 1600 3550 2    60   Input ~ 0
IR_LED_5
$Comp
L LED D6
U 1 1 5789834C
P 1400 4150
F 0 "D6" H 1400 4250 50  0000 C CNN
F 1 "LED" H 1400 4050 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 4150 50  0001 C CNN
F 3 "" H 1400 4150 50  0000 C CNN
	1    1400 4150
	-1   0    0    1   
$EndComp
Text GLabel 1600 4150 2    60   Input ~ 0
IR_LED_6
$Comp
L LED D7
U 1 1 57898360
P 1400 4750
F 0 "D7" H 1400 4850 50  0000 C CNN
F 1 "LED" H 1400 4650 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 4750 50  0001 C CNN
F 3 "" H 1400 4750 50  0000 C CNN
	1    1400 4750
	-1   0    0    1   
$EndComp
Text GLabel 1600 4750 2    60   Input ~ 0
IR_LED_7
$Comp
L LED D8
U 1 1 578C79D4
P 1400 5350
F 0 "D8" H 1400 5450 50  0000 C CNN
F 1 "LED" H 1400 5250 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 5350 50  0001 C CNN
F 3 "" H 1400 5350 50  0000 C CNN
	1    1400 5350
	-1   0    0    1   
$EndComp
Text GLabel 1600 5350 2    60   Input ~ 0
IR_LED_8
$Comp
L LED D9
U 1 1 578C79E8
P 1400 5950
F 0 "D9" H 1400 6050 50  0000 C CNN
F 1 "LED" H 1400 5850 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 5950 50  0001 C CNN
F 3 "" H 1400 5950 50  0000 C CNN
	1    1400 5950
	-1   0    0    1   
$EndComp
Text GLabel 1600 5950 2    60   Input ~ 0
IR_LED_9
$Comp
L LED D10
U 1 1 578C79FC
P 1400 6550
F 0 "D10" H 1400 6650 50  0000 C CNN
F 1 "LED" H 1400 6450 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 6550 50  0001 C CNN
F 3 "" H 1400 6550 50  0000 C CNN
	1    1400 6550
	-1   0    0    1   
$EndComp
Text GLabel 1600 6550 2    60   Input ~ 0
IR_LED_10
$Comp
L LED D11
U 1 1 578C7E10
P 1400 7150
F 0 "D11" H 1400 7250 50  0000 C CNN
F 1 "LED" H 1400 7050 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 1400 7150 50  0001 C CNN
F 3 "" H 1400 7150 50  0000 C CNN
	1    1400 7150
	-1   0    0    1   
$EndComp
Text GLabel 4700 2700 2    60   Input ~ 0
SPI_SCK
$Comp
L +3.3V #PWR01
U 1 1 578C8F52
P 4400 3450
F 0 "#PWR01" H 4400 3300 50  0001 C CNN
F 1 "+3.3V" H 4400 3590 50  0000 C CNN
F 2 "" H 4400 3450 50  0000 C CNN
F 3 "" H 4400 3450 50  0000 C CNN
	1    4400 3450
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR02
U 1 1 578C8F58
P 4100 3450
F 0 "#PWR02" H 4100 3200 50  0001 C CNN
F 1 "GND" H 4100 3300 50  0000 C CNN
F 2 "" H 4100 3450 50  0000 C CNN
F 3 "" H 4100 3450 50  0000 C CNN
	1    4100 3450
	1    0    0    -1  
$EndComp
Text GLabel 3800 2500 0    60   Input ~ 0
IR_LED_8
Text GLabel 3800 2600 0    60   Input ~ 0
IR_LED_9
Text GLabel 3800 2700 0    60   Input ~ 0
IR_LED_10
Text GLabel 6500 4300 0    60   Input ~ 0
Latch
NoConn ~ 3800 2800
NoConn ~ 3800 2900
$Comp
L 74HC595D U1
U 1 1 578CC6D2
P 4250 1600
F 0 "U1" H 4250 2450 60  0000 C CNN
F 1 "74HC595D" H 4300 1550 60  0000 C CNN
F 2 "footprints:74HC595D_SO16" H 4000 1600 60  0001 C CNN
F 3 "" H 4000 1600 60  0000 C CNN
	1    4250 1600
	1    0    0    -1  
$EndComp
$Comp
L 74HC595D U3
U 1 1 578CC74D
P 4250 3250
F 0 "U3" H 4250 4100 60  0000 C CNN
F 1 "74HC595D" H 4300 3200 60  0000 C CNN
F 2 "footprints:74HC595D_SO16" H 4000 3250 60  0001 C CNN
F 3 "" H 4000 3250 60  0000 C CNN
	1    4250 3250
	1    0    0    -1  
$EndComp
NoConn ~ 7800 2050
Text GLabel 7800 1850 2    60   Input ~ 0
Latch
Text GLabel 6600 3650 0    60   Input ~ 0
IR_PWM
$Comp
L +3V3 #PWR03
U 1 1 578F2608
P 6150 2650
F 0 "#PWR03" H 6150 2500 50  0001 C CNN
F 1 "+3V3" H 6150 2790 50  0000 C CNN
F 2 "" H 6150 2650 50  0000 C CNN
F 3 "" H 6150 2650 50  0000 C CNN
	1    6150 2650
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG04
U 1 1 578F2694
P 6400 2550
F 0 "#FLG04" H 6400 2645 50  0001 C CNN
F 1 "PWR_FLAG" H 6400 2730 50  0000 C CNN
F 2 "" H 6400 2550 50  0000 C CNN
F 3 "" H 6400 2550 50  0000 C CNN
	1    6400 2550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 5787C14A
P 6400 2550
F 0 "#PWR05" H 6400 2300 50  0001 C CNN
F 1 "GND" H 6400 2400 50  0000 C CNN
F 2 "" H 6400 2550 50  0000 C CNN
F 3 "" H 6400 2550 50  0000 C CNN
	1    6400 2550
	0    1    1    0   
$EndComp
$Comp
L PWR_FLAG #FLG06
U 1 1 578F289B
P 6150 2650
F 0 "#FLG06" H 6150 2745 50  0001 C CNN
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
L +3V3 #PWR07
U 1 1 578F5D3D
P 8100 3650
F 0 "#PWR07" H 8100 3500 50  0001 C CNN
F 1 "+3V3" H 8100 3790 50  0000 C CNN
F 2 "" H 8100 3650 50  0000 C CNN
F 3 "" H 8100 3650 50  0000 C CNN
	1    8100 3650
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR08
U 1 1 578F61E2
P 4400 1800
F 0 "#PWR08" H 4400 1650 50  0001 C CNN
F 1 "+3V3" H 4400 1940 50  0000 C CNN
F 2 "" H 4400 1800 50  0000 C CNN
F 3 "" H 4400 1800 50  0000 C CNN
	1    4400 1800
	-1   0    0    1   
$EndComp
$Comp
L SE8R01_module U4
U 1 1 578FCE1F
P 4500 4350
F 0 "U4" H 4300 4800 50  0000 L CNN
F 1 "SE8R01_module" H 4050 4050 50  0000 L CNN
F 2 "footprints:SE8R01_SMD" H 4200 4700 50  0001 L CNN
F 3 "" H 4550 4350 60  0000 C CNN
	1    4500 4350
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR09
U 1 1 578FD7B5
P 3700 3950
F 0 "#PWR09" H 3700 3800 50  0001 C CNN
F 1 "+3V3" H 3700 4090 50  0000 C CNN
F 2 "" H 3700 3950 50  0000 C CNN
F 3 "" H 3700 3950 50  0000 C CNN
	1    3700 3950
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR010
U 1 1 578FDB5C
P 3700 4250
F 0 "#PWR010" H 3700 4000 50  0001 C CNN
F 1 "GND" H 3700 4100 50  0000 C CNN
F 2 "" H 3700 4250 50  0000 C CNN
F 3 "" H 3700 4250 50  0000 C CNN
	1    3700 4250
	0    1    1    0   
$EndComp
$Comp
L C C1
U 1 1 578FDDE6
P 3700 4100
F 0 "C1" H 3725 4200 50  0000 L CNN
F 1 "100uf" H 3725 4000 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3738 3950 50  0001 C CNN
F 3 "" H 3700 4100 50  0000 C CNN
	1    3700 4100
	1    0    0    -1  
$EndComp
Text GLabel 3900 4400 0    60   Input ~ 0
SE8R01_CE
Text GLabel 3900 4550 0    60   Input ~ 0
SE8R01_CSN
Text GLabel 4750 4100 2    60   Input ~ 0
SPI_SCK
Text GLabel 6650 3850 0    60   Input ~ 0
SPI_MOSI
Text GLabel 4750 4400 2    60   Input ~ 0
SPI_MISO
Text GLabel 7800 1650 2    60   Input ~ 0
SPI_MISO
Text GLabel 4750 4550 2    60   Input ~ 0
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
Text GLabel 4700 2800 2    60   Input ~ 0
Latch
Text GLabel 4750 4250 2    60   Input ~ 0
SPI_MOSI
Text GLabel 4700 1150 2    60   Input ~ 0
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
L GND #PWR011
U 1 1 5790E39C
P 7950 4650
F 0 "#PWR011" H 7950 4400 50  0001 C CNN
F 1 "GND" H 7950 4500 50  0000 C CNN
F 2 "" H 7950 4650 50  0000 C CNN
F 3 "" H 7950 4650 50  0000 C CNN
	1    7950 4650
	0    1    1    0   
$EndComp
$Comp
L Via P10
U 1 1 5790E41F
P 8050 4450
F 0 "P10" H 8050 4550 50  0000 C CNN
F 1 "Via" V 8150 4450 50  0000 C CNN
F 2 "footprints:Via" H 8050 4450 50  0001 C CNN
F 3 "" H 8050 4450 50  0000 C CNN
	1    8050 4450
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR012
U 1 1 57910C6F
P 6700 2450
F 0 "#PWR012" H 6700 2300 50  0001 C CNN
F 1 "+5V" H 6700 2590 50  0000 C CNN
F 2 "" H 6700 2450 50  0000 C CNN
F 3 "" H 6700 2450 50  0000 C CNN
	1    6700 2450
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG013
U 1 1 57910CFC
P 6700 2450
F 0 "#FLG013" H 6700 2545 50  0001 C CNN
F 1 "PWR_FLAG" H 6700 2630 50  0000 C CNN
F 2 "" H 6700 2450 50  0000 C CNN
F 3 "" H 6700 2450 50  0000 C CNN
	1    6700 2450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR014
U 1 1 5791C22D
P 1000 700
F 0 "#PWR014" H 1000 550 50  0001 C CNN
F 1 "+5V" H 1000 840 50  0000 C CNN
F 2 "" H 1000 700 50  0000 C CNN
F 3 "" H 1000 700 50  0000 C CNN
	1    1000 700 
	0    -1   -1   0   
$EndComp
Text GLabel 4700 1350 2    60   Input ~ 0
Cascade
Text GLabel 4700 2500 2    60   Input ~ 0
Cascade
Text GLabel 6550 4750 0    60   Input ~ 0
Cascade
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
L +5V #PWR015
U 1 1 579691C2
P 7600 3900
F 0 "#PWR015" H 7600 3750 50  0001 C CNN
F 1 "+5V" H 7600 4040 50  0000 C CNN
F 2 "" H 7600 3900 50  0000 C CNN
F 3 "" H 7600 3900 50  0000 C CNN
	1    7600 3900
	0    -1   -1   0   
$EndComp
NoConn ~ 6950 1750
NoConn ~ 6950 1850
Text GLabel 5400 950  2    60   Input ~ 0
IR_PWM
$Comp
L +3V3 #PWR016
U 1 1 57A11085
P 5400 650
F 0 "#PWR016" H 5400 500 50  0001 C CNN
F 1 "+3V3" H 5400 790 50  0000 C CNN
F 2 "" H 5400 650 50  0000 C CNN
F 3 "" H 5400 650 50  0000 C CNN
	1    5400 650 
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 57A11104
P 5400 800
F 0 "R4" V 5480 800 50  0000 C CNN
F 1 "10K" V 5400 800 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" V 5330 800 50  0001 C CNN
F 3 "" H 5400 800 50  0000 C CNN
	1    5400 800 
	1    0    0    -1  
$EndComp
Text GLabel 5400 2600 2    60   Input ~ 0
IR_PWM
$Comp
L +3V3 #PWR017
U 1 1 57A116F2
P 5400 2300
F 0 "#PWR017" H 5400 2150 50  0001 C CNN
F 1 "+3V3" H 5400 2440 50  0000 C CNN
F 2 "" H 5400 2300 50  0000 C CNN
F 3 "" H 5400 2300 50  0000 C CNN
	1    5400 2300
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 57A116F8
P 5400 2450
F 0 "R5" V 5480 2450 50  0000 C CNN
F 1 "10K" V 5400 2450 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" V 5330 2450 50  0001 C CNN
F 3 "" H 5400 2450 50  0000 C CNN
	1    5400 2450
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 57A11C3B
P 5350 2900
F 0 "R3" V 5430 2900 50  0000 C CNN
F 1 "10K" V 5350 2900 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" V 5280 2900 50  0001 C CNN
F 3 "" H 5350 2900 50  0000 C CNN
	1    5350 2900
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 57A11D31
P 5350 1250
F 0 "R2" V 5430 1250 50  0000 C CNN
F 1 "10K" V 5350 1250 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" V 5280 1250 50  0001 C CNN
F 3 "" H 5350 1250 50  0000 C CNN
	1    5350 1250
	0    1    1    0   
$EndComp
$Comp
L GND #PWR018
U 1 1 57A11E8E
P 5500 1250
F 0 "#PWR018" H 5500 1000 50  0001 C CNN
F 1 "GND" H 5500 1100 50  0000 C CNN
F 2 "" H 5500 1250 50  0000 C CNN
F 3 "" H 5500 1250 50  0000 C CNN
	1    5500 1250
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR019
U 1 1 57A11F50
P 5500 2900
F 0 "#PWR019" H 5500 2650 50  0001 C CNN
F 1 "GND" H 5500 2750 50  0000 C CNN
F 2 "" H 5500 2900 50  0000 C CNN
F 3 "" H 5500 2900 50  0000 C CNN
	1    5500 2900
	0    -1   -1   0   
$EndComp
Text GLabel 1600 1150 2    60   Input ~ 0
IR_LED_1
Text GLabel 1600 7150 2    60   Input ~ 0
IR_LED_DATA
$Comp
L R R1
U 1 1 57A138ED
P 1150 700
F 0 "R1" V 1230 700 50  0000 C CNN
F 1 "50" V 1150 700 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" V 1080 700 50  0001 C CNN
F 3 "" H 1150 700 50  0000 C CNN
	1    1150 700 
	0    1    1    0   
$EndComp
Text GLabel 1300 700  2    60   Input ~ 0
IR_VCC
Text GLabel 1200 1150 0    60   Input ~ 0
IR_VCC
Text GLabel 1200 1750 0    60   Input ~ 0
IR_VCC
Text GLabel 1200 2350 0    60   Input ~ 0
IR_VCC
Text GLabel 1200 2950 0    60   Input ~ 0
IR_VCC
Text GLabel 1200 3550 0    60   Input ~ 0
IR_VCC
Text GLabel 1200 4150 0    60   Input ~ 0
IR_VCC
Text GLabel 1200 4750 0    60   Input ~ 0
IR_VCC
Text GLabel 1200 5350 0    60   Input ~ 0
IR_VCC
Text GLabel 1200 5950 0    60   Input ~ 0
IR_VCC
Text GLabel 1200 6550 0    60   Input ~ 0
IR_VCC
Text GLabel 1200 7150 0    60   Input ~ 0
IR_VCC
Text GLabel 3800 3000 0    60   Input ~ 0
IR_LED_DATA
NoConn ~ 4700 3000
NoConn ~ 4700 2400
NoConn ~ 4700 750 
$Comp
L GND #PWR020
U 1 1 5787C121
P 4100 1800
F 0 "#PWR020" H 4100 1550 50  0001 C CNN
F 1 "GND" H 4100 1650 50  0000 C CNN
F 2 "" H 4100 1800 50  0000 C CNN
F 3 "" H 4100 1800 50  0000 C CNN
	1    4100 1800
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 57A21B20
P 4250 1800
F 0 "C2" H 4275 1900 50  0000 L CNN
F 1 "100uf" H 4275 1700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4288 1650 50  0001 C CNN
F 3 "" H 4250 1800 50  0000 C CNN
	1    4250 1800
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 57A21E8C
P 4250 3450
F 0 "C3" H 4275 3550 50  0000 L CNN
F 1 "100uf" H 4275 3350 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4288 3300 50  0001 C CNN
F 3 "" H 4250 3450 50  0000 C CNN
	1    4250 3450
	0    1    1    0   
$EndComp
$Comp
L Via P4
U 1 1 57A2615C
P 2000 1350
F 0 "P4" H 2000 1450 50  0000 C CNN
F 1 "Via" V 2100 1350 50  0000 C CNN
F 2 "footprints:Via" H 2000 1350 50  0001 C CNN
F 3 "" H 2000 1350 50  0000 C CNN
	1    2000 1350
	1    0    0    -1  
$EndComp
$Comp
L Via P6
U 1 1 57A26746
P 2000 1950
F 0 "P6" H 2000 2050 50  0000 C CNN
F 1 "Via" V 2100 1950 50  0000 C CNN
F 2 "footprints:Via" H 2000 1950 50  0001 C CNN
F 3 "" H 2000 1950 50  0000 C CNN
	1    2000 1950
	1    0    0    -1  
$EndComp
$Comp
L Via P11
U 1 1 57A267BD
P 2000 2550
F 0 "P11" H 2000 2650 50  0000 C CNN
F 1 "Via" V 2100 2550 50  0000 C CNN
F 2 "footprints:Via" H 2000 2550 50  0001 C CNN
F 3 "" H 2000 2550 50  0000 C CNN
	1    2000 2550
	1    0    0    -1  
$EndComp
$Comp
L Via P12
U 1 1 57A26909
P 2000 3150
F 0 "P12" H 2000 3250 50  0000 C CNN
F 1 "Via" V 2100 3150 50  0000 C CNN
F 2 "footprints:Via" H 2000 3150 50  0001 C CNN
F 3 "" H 2000 3150 50  0000 C CNN
	1    2000 3150
	1    0    0    -1  
$EndComp
$Comp
L Via P13
U 1 1 57A2699C
P 2000 3750
F 0 "P13" H 2000 3850 50  0000 C CNN
F 1 "Via" V 2100 3750 50  0000 C CNN
F 2 "footprints:Via" H 2000 3750 50  0001 C CNN
F 3 "" H 2000 3750 50  0000 C CNN
	1    2000 3750
	1    0    0    -1  
$EndComp
$Comp
L Via P15
U 1 1 57A26A38
P 2000 4350
F 0 "P15" H 2000 4450 50  0000 C CNN
F 1 "Via" V 2100 4350 50  0000 C CNN
F 2 "footprints:Via" H 2000 4350 50  0001 C CNN
F 3 "" H 2000 4350 50  0000 C CNN
	1    2000 4350
	1    0    0    -1  
$EndComp
$Comp
L Via P16
U 1 1 57A26AD1
P 2000 4950
F 0 "P16" H 2000 5050 50  0000 C CNN
F 1 "Via" V 2100 4950 50  0000 C CNN
F 2 "footprints:Via" H 2000 4950 50  0001 C CNN
F 3 "" H 2000 4950 50  0000 C CNN
	1    2000 4950
	1    0    0    -1  
$EndComp
$Comp
L Via P18
U 1 1 57A26CD5
P 2000 5550
F 0 "P18" H 2000 5650 50  0000 C CNN
F 1 "Via" V 2100 5550 50  0000 C CNN
F 2 "footprints:Via" H 2000 5550 50  0001 C CNN
F 3 "" H 2000 5550 50  0000 C CNN
	1    2000 5550
	1    0    0    -1  
$EndComp
$Comp
L Via P19
U 1 1 57A26D7C
P 2000 6150
F 0 "P19" H 2000 6250 50  0000 C CNN
F 1 "Via" V 2100 6150 50  0000 C CNN
F 2 "footprints:Via" H 2000 6150 50  0001 C CNN
F 3 "" H 2000 6150 50  0000 C CNN
	1    2000 6150
	1    0    0    -1  
$EndComp
$Comp
L Via P20
U 1 1 57A26E26
P 2000 6750
F 0 "P20" H 2000 6850 50  0000 C CNN
F 1 "Via" V 2100 6750 50  0000 C CNN
F 2 "footprints:Via" H 2000 6750 50  0001 C CNN
F 3 "" H 2000 6750 50  0000 C CNN
	1    2000 6750
	1    0    0    -1  
$EndComp
$Comp
L Via P21
U 1 1 57A26EDF
P 2000 7350
F 0 "P21" H 2000 7450 50  0000 C CNN
F 1 "Via" V 2100 7350 50  0000 C CNN
F 2 "footprints:Via" H 2000 7350 50  0001 C CNN
F 3 "" H 2000 7350 50  0000 C CNN
	1    2000 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 5550 1600 5350
Wire Wire Line
	1800 5550 1600 5550
Wire Wire Line
	1600 4950 1600 4750
Wire Wire Line
	1800 4950 1600 4950
Wire Wire Line
	1600 4350 1600 4150
Wire Wire Line
	1800 4350 1600 4350
Wire Wire Line
	1600 3750 1600 3550
Wire Wire Line
	1800 3750 1600 3750
Wire Wire Line
	1600 3150 1600 2950
Wire Wire Line
	1800 3150 1600 3150
Wire Wire Line
	1600 2550 1600 2350
Wire Wire Line
	1800 2550 1600 2550
Wire Wire Line
	1600 1950 1600 1750
Wire Wire Line
	1800 1950 1600 1950
Wire Wire Line
	1600 1350 1600 1150
Wire Wire Line
	1800 1350 1600 1350
Wire Wire Line
	8050 4650 7950 4650
Wire Wire Line
	3900 4250 3700 4250
Wire Wire Line
	5200 1250 4700 1250
Wire Wire Line
	5400 2600 4700 2600
Wire Wire Line
	5400 950  4700 950 
Wire Wire Line
	8100 3900 8100 3650
Wire Wire Line
	3700 3950 3900 3950
Wire Wire Line
	6950 2650 6150 2650
Wire Wire Line
	3900 3950 3900 4100
Wire Wire Line
	6400 2550 6950 2550
Wire Wire Line
	6950 2450 6700 2450
Wire Wire Line
	6550 4750 6900 4750
Connection ~ 6700 4750
Wire Wire Line
	6500 4300 6900 4300
Connection ~ 6700 4300
Wire Wire Line
	1800 6150 1600 6150
Wire Wire Line
	1600 6150 1600 5950
Wire Wire Line
	1800 6750 1600 6750
Wire Wire Line
	1600 6750 1600 6550
Wire Wire Line
	1800 7350 1600 7350
Wire Wire Line
	1600 7350 1600 7150
Wire Wire Line
	5200 2900 4700 2900
NoConn ~ 3800 750 
$EndSCHEMATC
