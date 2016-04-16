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
LIBS:STM32F103C8T6
LIBS:UsbDevice-cache
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
L NRF24L01-BRKOUT U?
U 1 1 5711D315
P 5250 3250
F 0 "U?" H 5200 3250 50  0000 L CNN
F 1 "NRF24L01-BRKOUT" H 4900 2700 50  0000 L CNN
F 2 "NRF24L01-BKKOUT" H 5250 3250 50  0001 L CNN
F 3 "" H 5250 3250 60  0000 C CNN
	1    5250 3250
	1    0    0    -1  
$EndComp
$Comp
L NRF24L01-BRKOUT U?
U 1 1 5711D3FC
P 5250 1950
F 0 "U?" H 5250 1950 50  0000 L CNN
F 1 "NRF24L01-BRKOUT" H 4900 1400 50  0000 L CNN
F 2 "NRF24L01-BKKOUT" H 5250 1950 50  0001 L CNN
F 3 "" H 5250 1950 60  0000 C CNN
	1    5250 1950
	1    0    0    -1  
$EndComp
Text GLabel 3650 2050 2    60   Input ~ 0
MOSI
$Comp
L STM32F103C8T6 U?
U 1 1 5711D974
P 2500 2650
F 0 "U?" H 2500 2650 50  0001 L CNN
F 1 "STM32F103C8T6" H 2500 2650 50  0001 L CNN
F 2 "LQFP48_7X7MM_0.5MMPP" H 2500 2650 50  0001 L CNN
F 3 "" H 2500 2650 60  0000 C CNN
	1    2500 2650
	1    0    0    -1  
$EndComp
$EndSCHEMATC
