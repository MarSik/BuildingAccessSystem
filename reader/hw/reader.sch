EESchema Schematic File Version 4
LIBS:reader-cache
EELAYER 26 0
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
L reader-rescue:MKE04P24M48SF0-RESCUE-reader U103
U 1 1 591A19D8
P 5850 3950
F 0 "U103" H 5850 4937 60  0000 C CNN
F 1 "MKE04P24M48SF0" H 5850 4831 60  0000 C CNN
F 2 "smd-normal:SOIC-20_7.5x12.8mm_Pitch1.27mm" H 5700 3650 60  0001 C CNN
F 3 "" H 5700 3650 60  0001 C CNN
	1    5850 3950
	1    0    0    -1  
$EndComp
$Comp
L connectors:Wago_233-502 P101
U 1 1 591A1AB9
P 1750 5400
F 0 "P101" H 1828 5441 50  0000 L CNN
F 1 "Wago_233-502" H 1828 5350 50  0000 L CNN
F 2 "th-npth:Wire-2_Wago-233-P2.5mm" H 1750 5400 60  0001 C CNN
F 3 "" H 1750 5400 60  0000 C CNN
	1    1750 5400
	-1   0    0    -1  
$EndComp
$Comp
L connectors:Wago_233-502 P102
U 1 1 591A1B3F
P 1750 6550
F 0 "P102" H 1828 6591 50  0000 L CNN
F 1 "Wago_233-502" H 1828 6500 50  0000 L CNN
F 2 "th-npth:Wire-2_Wago-233-P2.5mm" H 1750 6550 60  0001 C CNN
F 3 "" H 1750 6550 60  0000 C CNN
	1    1750 6550
	-1   0    0    -1  
$EndComp
$Comp
L connectors:CONN_01X08 P105
U 1 1 591A1E5B
P 10400 4000
F 0 "P105" H 10478 4041 50  0000 L CNN
F 1 "CONN_01X08" H 10478 3950 50  0000 L CNN
F 2 "th-normal:Pin_Header_Straight_1x08" H 10400 4000 60  0001 C CNN
F 3 "" H 10400 4000 60  0000 C CNN
	1    10400 4000
	1    0    0    -1  
$EndComp
$Comp
L simple:C C104
U 1 1 591A1F28
P 2300 7300
F 0 "C104" H 2415 7346 50  0000 L CNN
F 1 "C" H 2415 7255 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 2338 7150 30  0001 C CNN
F 3 "" H 2300 7300 60  0000 C CNN
	1    2300 7300
	1    0    0    -1  
$EndComp
$Comp
L power:LP2992AIM5-3.3CT-ND U101
U 1 1 591A260B
P 2500 1450
F 0 "U101" H 2500 1937 60  0000 C CNN
F 1 "LP2992AIM5-3.3CT-ND" H 2500 1831 60  0000 C CNN
F 2 "smd-handsolder:SOT-23-5_HandSoldering" H 2800 1200 50  0001 C CNN
F 3 "" H 2500 1350 60  0000 C CNN
	1    2500 1450
	1    0    0    -1  
$EndComp
$Comp
L simple:C C101
U 1 1 591A26E0
P 1450 1450
F 0 "C101" H 1564 1496 50  0000 L CNN
F 1 "C" H 1564 1405 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 1488 1300 30  0001 C CNN
F 3 "" H 1450 1450 60  0000 C CNN
	1    1450 1450
	-1   0    0    -1  
$EndComp
$Comp
L simple:C C102
U 1 1 591A272E
P 1800 1750
F 0 "C102" H 1914 1796 50  0000 L CNN
F 1 "C" H 1914 1705 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 1838 1600 30  0001 C CNN
F 3 "" H 1800 1750 60  0000 C CNN
	1    1800 1750
	-1   0    0    -1  
$EndComp
$Comp
L simple:C C106
U 1 1 591A27D8
P 3350 1450
F 0 "C106" H 3464 1496 50  0000 L CNN
F 1 "C" H 3464 1405 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 3388 1300 30  0001 C CNN
F 3 "" H 3350 1450 60  0000 C CNN
	1    3350 1450
	-1   0    0    -1  
$EndComp
$Comp
L ic:SN75176BDR U102
U 1 1 591A2A34
P 2850 6550
F 0 "U102" H 2700 6950 60  0000 R CNN
F 1 "SN75176BDR" H 2700 7050 60  0000 R CNN
F 2 "smd-normal:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 2850 6550 60  0001 C CNN
F 3 "" H 2850 6550 60  0000 C CNN
	1    2850 6550
	-1   0    0    -1  
$EndComp
$Comp
L simple:R R101
U 1 1 591A2BE3
P 2150 6550
F 0 "R101" H 2220 6596 50  0000 L CNN
F 1 "100" V 2150 6550 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 2080 6550 30  0001 C CNN
F 3 "" H 2150 6550 30  0000 C CNN
	1    2150 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 6500 2000 6500
Wire Wire Line
	2000 6500 2000 6400
Wire Wire Line
	2000 6400 2450 6400
Wire Wire Line
	2450 6400 2450 6450
Connection ~ 2150 6400
Wire Wire Line
	1950 6600 1950 6700
Wire Wire Line
	1950 6700 2450 6700
Wire Wire Line
	2450 6700 2450 6650
Connection ~ 2150 6700
$Comp
L mspower:GND #PWR01
U 1 1 591A2EEE
P 2000 5550
F 0 "#PWR01" H 2000 5300 50  0001 C CNN
F 1 "GND" H 2005 5377 50  0000 C CNN
F 2 "" H 2000 5550 60  0000 C CNN
F 3 "" H 2000 5550 60  0000 C CNN
	1    2000 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 5450 2000 5450
Wire Wire Line
	2000 5450 2000 5550
Wire Wire Line
	2000 5350 1950 5350
Wire Wire Line
	2000 5200 2000 5350
$Comp
L simple:C C103
U 1 1 591A2FC4
P 2250 5400
F 0 "C103" H 2365 5446 50  0000 L CNN
F 1 "C" H 2365 5355 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 2288 5250 30  0001 C CNN
F 3 "" H 2250 5400 60  0000 C CNN
	1    2250 5400
	1    0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR02
U 1 1 591A301B
P 2250 5550
F 0 "#PWR02" H 2250 5300 50  0001 C CNN
F 1 "GND" H 2255 5377 50  0000 C CNN
F 2 "" H 2250 5550 60  0000 C CNN
F 3 "" H 2250 5550 60  0000 C CNN
	1    2250 5550
	1    0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR03
U 1 1 591A310F
P 2650 5550
F 0 "#PWR03" H 2650 5300 50  0001 C CNN
F 1 "GND" H 2655 5377 50  0000 C CNN
F 2 "" H 2650 5550 60  0000 C CNN
F 3 "" H 2650 5550 60  0000 C CNN
	1    2650 5550
	1    0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR04
U 1 1 591A36EC
P 2300 7450
F 0 "#PWR04" H 2300 7200 50  0001 C CNN
F 1 "GND" H 2305 7277 50  0000 C CNN
F 2 "" H 2300 7450 60  0000 C CNN
F 3 "" H 2300 7450 60  0000 C CNN
	1    2300 7450
	1    0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR05
U 1 1 591A3757
P 2850 6850
F 0 "#PWR05" H 2850 6600 50  0001 C CNN
F 1 "GND" H 2855 6677 50  0000 C CNN
F 2 "" H 2850 6850 60  0000 C CNN
F 3 "" H 2850 6850 60  0000 C CNN
	1    2850 6850
	1    0    0    -1  
$EndComp
Text Label 7000 4650 0    60   ~ 0
CPU_TX
Text Label 7000 4550 0    60   ~ 0
CPU_RX
Text Label 4400 4550 0    60   ~ 0
MISO
Text Label 4400 4350 0    60   ~ 0
MOSI
Text Label 4400 4450 0    60   ~ 0
SCK
Wire Wire Line
	6850 4650 7450 4650
Wire Wire Line
	4850 4350 4300 4350
Wire Wire Line
	4850 4450 4300 4450
Wire Wire Line
	4850 4550 4300 4550
$Comp
L connectors:CONN_02X05 P103
U 1 1 591B7E46
P 1900 3650
F 0 "P103" H 1900 4065 50  0000 C CNN
F 1 "CONN_02X05" H 1900 3974 50  0000 C CNN
F 2 "th-normal:Pin_Header_Straight_2x05" H 1900 2450 60  0001 C CNN
F 3 "" H 1900 2450 60  0000 C CNN
	1    1900 3650
	1    0    0    -1  
$EndComp
$Comp
L mspower:+3.3V #PWR06
U 1 1 591B7F21
P 3600 1200
F 0 "#PWR06" H 3600 1050 50  0001 C CNN
F 1 "+3.3V" H 3615 1373 50  0000 C CNN
F 2 "" H 3600 1200 60  0000 C CNN
F 3 "" H 3600 1200 60  0000 C CNN
	1    3600 1200
	1    0    0    -1  
$EndComp
$Comp
L mspower:+3.3V #PWR07
U 1 1 591B7F4F
P 1450 3350
F 0 "#PWR07" H 1450 3200 50  0001 C CNN
F 1 "+3.3V" H 1465 3523 50  0000 C CNN
F 2 "" H 1450 3350 60  0000 C CNN
F 3 "" H 1450 3350 60  0000 C CNN
	1    1450 3350
	1    0    0    -1  
$EndComp
Text Label 7000 4150 0    60   ~ 0
SWD_DIO
Text Label 2350 3450 0    60   ~ 0
SWD_DIO
Text Label 7000 4250 0    60   ~ 0
SWD_CLK
Text Label 2350 3550 0    60   ~ 0
SWD_CLK
Text Label 7000 3950 0    60   ~ 0
~RESET
Text Label 2350 3850 0    60   ~ 0
~RESET
$Comp
L mspower:GND #PWR08
U 1 1 591B80CB
P 1450 3950
F 0 "#PWR08" H 1450 3700 50  0001 C CNN
F 1 "GND" H 1455 3777 50  0000 C CNN
F 2 "" H 1450 3950 60  0000 C CNN
F 3 "" H 1450 3950 60  0000 C CNN
	1    1450 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 3350 1450 3450
Wire Wire Line
	1450 3450 1650 3450
Wire Wire Line
	2150 3450 2800 3450
Wire Wire Line
	2150 3550 2800 3550
Wire Wire Line
	1650 3550 1450 3550
Wire Wire Line
	1450 3550 1450 3950
Wire Wire Line
	1650 3650 1450 3650
Connection ~ 1450 3650
NoConn ~ 2150 3650
NoConn ~ 2150 3750
NoConn ~ 1650 3750
NoConn ~ 1650 3850
Wire Wire Line
	2150 3850 2800 3850
Wire Wire Line
	6850 3950 7450 3950
Wire Wire Line
	6850 4150 7450 4150
Wire Wire Line
	6850 4250 7950 4250
$Comp
L mspower:VCC #PWR09
U 1 1 591B85E5
P 950 1200
F 0 "#PWR09" H 950 1050 50  0001 C CNN
F 1 "VCC" H 967 1373 50  0000 C CNN
F 2 "" H 950 1200 60  0000 C CNN
F 3 "" H 950 1200 60  0000 C CNN
	1    950  1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 1250 1450 1300
Wire Wire Line
	950  1250 1900 1250
Connection ~ 1450 1250
Wire Wire Line
	1600 1250 1600 1350
Wire Wire Line
	1600 1350 1900 1350
Connection ~ 1600 1250
Wire Wire Line
	1900 1550 1800 1550
Wire Wire Line
	1800 1550 1800 1600
Wire Wire Line
	1450 1600 1450 2000
Wire Wire Line
	1450 2000 3350 2000
Wire Wire Line
	1800 1900 1800 2000
Connection ~ 1800 2000
Wire Wire Line
	2500 1850 2500 2050
Wire Wire Line
	3350 2000 3350 1600
Wire Wire Line
	3100 1250 3600 1250
Wire Wire Line
	3350 1250 3350 1300
Wire Wire Line
	3600 1250 3600 1200
Connection ~ 3350 1250
$Comp
L mspower:GND #PWR010
U 1 1 591B892E
P 2500 2050
F 0 "#PWR010" H 2500 1800 50  0001 C CNN
F 1 "GND" H 2505 1877 50  0000 C CNN
F 2 "" H 2500 2050 60  0000 C CNN
F 3 "" H 2500 2050 60  0000 C CNN
	1    2500 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  1200 950  1250
$Comp
L mspower:GND #PWR011
U 1 1 591B8BC6
P 4700 3400
F 0 "#PWR011" H 4700 3150 50  0001 C CNN
F 1 "GND" H 4705 3227 50  0000 C CNN
F 2 "" H 4700 3400 60  0000 C CNN
F 3 "" H 4700 3400 60  0000 C CNN
	1    4700 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 3350 4700 3350
Wire Wire Line
	4700 3350 4700 3400
$Comp
L mspower:+3.3V #PWR012
U 1 1 591B8C5A
P 4700 3200
F 0 "#PWR012" H 4700 3050 50  0001 C CNN
F 1 "+3.3V" H 4715 3373 50  0000 C CNN
F 2 "" H 4700 3200 60  0000 C CNN
F 3 "" H 4700 3200 60  0000 C CNN
	1    4700 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 3200 4700 3250
Wire Wire Line
	4700 3250 4850 3250
$Comp
L mspower:VCC #PWR013
U 1 1 591B9127
P 2000 5200
F 0 "#PWR013" H 2000 5050 50  0001 C CNN
F 1 "VCC" H 2017 5373 50  0000 C CNN
F 2 "" H 2000 5200 60  0000 C CNN
F 3 "" H 2000 5200 60  0000 C CNN
	1    2000 5200
	1    0    0    -1  
$EndComp
$Comp
L mspower:VCC #PWR014
U 1 1 591B91AF
P 2250 5200
F 0 "#PWR014" H 2250 5050 50  0001 C CNN
F 1 "VCC" H 2267 5373 50  0000 C CNN
F 2 "" H 2250 5200 60  0000 C CNN
F 3 "" H 2250 5200 60  0000 C CNN
	1    2250 5200
	1    0    0    -1  
$EndComp
$Comp
L mspower:VCC #PWR015
U 1 1 591B91D6
P 2650 5200
F 0 "#PWR015" H 2650 5050 50  0001 C CNN
F 1 "VCC" H 2667 5373 50  0000 C CNN
F 2 "" H 2650 5200 60  0000 C CNN
F 3 "" H 2650 5200 60  0000 C CNN
	1    2650 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 5200 2250 5250
Wire Wire Line
	2650 5200 2650 5250
$Comp
L simple:R R104
U 1 1 591B949B
P 7550 3750
F 0 "R104" V 7450 3750 50  0000 C CNN
F 1 "10k" V 7550 3750 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 7480 3750 30  0001 C CNN
F 3 "" H 7550 3750 30  0000 C CNN
	1    7550 3750
	0    1    1    0   
$EndComp
$Comp
L mspower:+3.3V #PWR016
U 1 1 591B9529
P 8500 3750
F 0 "#PWR016" H 8500 3600 50  0001 C CNN
F 1 "+3.3V" H 8515 3923 50  0000 C CNN
F 2 "" H 8500 3750 60  0000 C CNN
F 3 "" H 8500 3750 60  0000 C CNN
	1    8500 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 3950 7350 3750
Wire Wire Line
	7350 3750 7400 3750
Connection ~ 7350 3950
Wire Wire Line
	7700 3750 8500 3750
Text Label 7000 4450 0    60   ~ 0
TX_EN
Text Label 3450 6550 0    60   ~ 0
TX_EN
Text Label 3450 6350 0    60   ~ 0
CPU_TX
Text Label 3450 6750 0    60   ~ 0
CPU_RX
Wire Wire Line
	3150 6750 3900 6750
Wire Wire Line
	3150 6350 3900 6350
Wire Wire Line
	3150 6450 3250 6450
Wire Wire Line
	3250 6450 3250 6650
Wire Wire Line
	3250 6550 4200 6550
Wire Wire Line
	3250 6650 3150 6650
Connection ~ 3250 6550
$Comp
L simple:R R103
U 1 1 591B9D0B
P 4050 6800
F 0 "R103" H 4120 6846 50  0000 L CNN
F 1 "R" H 4120 6755 50  0000 L CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 3980 6800 30  0001 C CNN
F 3 "" H 4050 6800 30  0000 C CNN
	1    4050 6800
	1    0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR017
U 1 1 591B9F1D
P 4050 6950
F 0 "#PWR017" H 4050 6700 50  0001 C CNN
F 1 "GND" H 4055 6777 50  0000 C CNN
F 2 "" H 4050 6950 60  0000 C CNN
F 3 "" H 4050 6950 60  0000 C CNN
	1    4050 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 6650 4050 6550
Connection ~ 4050 6550
Wire Wire Line
	6850 4550 7450 4550
Wire Wire Line
	6850 4450 7450 4450
$Comp
L simple:LED D101
U 1 1 591BBBD3
P 9050 1950
F 0 "D101" H 9050 1705 50  0000 C CNN
F 1 "LED" H 9050 1796 50  0000 C CNN
F 2 "smd-normal:LED-1206" H 9050 1950 60  0001 C CNN
F 3 "" H 9050 1950 60  0000 C CNN
	1    9050 1950
	1    0    0    1   
$EndComp
$Comp
L simple:LED D102
U 1 1 591BBCC5
P 9050 2400
F 0 "D102" H 9050 2155 50  0000 C CNN
F 1 "LED" H 9050 2246 50  0000 C CNN
F 2 "smd-normal:LED-1206" H 9050 2400 60  0001 C CNN
F 3 "" H 9050 2400 60  0000 C CNN
	1    9050 2400
	1    0    0    1   
$EndComp
$Comp
L simple:R R108
U 1 1 591C09A0
P 8650 1950
F 0 "R108" V 8443 1950 50  0000 C CNN
F 1 "R" V 8534 1950 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 8580 1950 30  0001 C CNN
F 3 "" H 8650 1950 30  0000 C CNN
	1    8650 1950
	0    1    1    0   
$EndComp
$Comp
L simple:R R109
U 1 1 591C0A26
P 8650 2400
F 0 "R109" V 8443 2400 50  0000 C CNN
F 1 "R" V 8534 2400 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 8580 2400 30  0001 C CNN
F 3 "" H 8650 2400 30  0000 C CNN
	1    8650 2400
	0    1    1    0   
$EndComp
$Comp
L transistors:MMBT2222A Q101
U 1 1 591C0AAE
P 7600 2050
F 0 "Q101" V 7928 2050 50  0000 C CNN
F 1 "MMBT2222A" V 7837 2050 50  0000 C CNN
F 2 "smd-handsolder:SOT-23_HandSoldering" H 7800 2150 29  0001 C CNN
F 3 "" H 7600 2050 60  0000 C CNN
	1    7600 2050
	0    1    -1   0   
$EndComp
$Comp
L transistors:MMBT2222A Q102
U 1 1 591C0B6C
P 8200 2500
F 0 "Q102" V 8528 2500 50  0000 C CNN
F 1 "MMBT2222A" V 8437 2500 50  0000 C CNN
F 2 "smd-handsolder:SOT-23_HandSoldering" H 8400 2600 29  0001 C CNN
F 3 "" H 8200 2500 60  0000 C CNN
	1    8200 2500
	0    1    -1   0   
$EndComp
Wire Wire Line
	8800 1950 8850 1950
Wire Wire Line
	8800 2400 8850 2400
$Comp
L mspower:VCC #PWR018
U 1 1 591C0EC3
P 9450 1850
F 0 "#PWR018" H 9450 1700 50  0001 C CNN
F 1 "VCC" H 9467 2023 50  0000 C CNN
F 2 "" H 9450 1850 60  0000 C CNN
F 3 "" H 9450 1850 60  0000 C CNN
	1    9450 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 1950 9450 1950
Wire Wire Line
	9450 1850 9450 2400
Wire Wire Line
	9450 2400 9250 2400
Connection ~ 9450 1950
Wire Wire Line
	8500 1950 7800 1950
Wire Wire Line
	8500 2400 8400 2400
$Comp
L simple:R R105
U 1 1 591C10BA
P 7600 2950
F 0 "R105" H 7670 2996 50  0000 L CNN
F 1 "10k" H 7670 2905 50  0000 L CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 7530 2950 30  0001 C CNN
F 3 "" H 7600 2950 30  0000 C CNN
	1    7600 2950
	1    0    0    -1  
$EndComp
$Comp
L simple:R R107
U 1 1 591C1132
P 8200 2950
F 0 "R107" H 8270 2996 50  0000 L CNN
F 1 "10k" H 8270 2905 50  0000 L CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 8130 2950 30  0001 C CNN
F 3 "" H 8200 2950 30  0000 C CNN
	1    8200 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 2250 7600 2800
Wire Wire Line
	8200 2700 8200 2800
Wire Wire Line
	7600 3100 7600 3450
Wire Wire Line
	8200 3100 8200 3550
$Comp
L mspower:GND #PWR019
U 1 1 591C1366
P 7200 2050
F 0 "#PWR019" H 7200 1800 50  0001 C CNN
F 1 "GND" H 7205 1877 50  0000 C CNN
F 2 "" H 7200 2050 60  0000 C CNN
F 3 "" H 7200 2050 60  0000 C CNN
	1    7200 2050
	1    0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR020
U 1 1 591C13A8
P 7850 2500
F 0 "#PWR020" H 7850 2250 50  0001 C CNN
F 1 "GND" H 7855 2327 50  0000 C CNN
F 2 "" H 7850 2500 60  0000 C CNN
F 3 "" H 7850 2500 60  0000 C CNN
	1    7850 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 2500 7850 2400
Wire Wire Line
	7850 2400 8000 2400
Wire Wire Line
	7200 2050 7200 1950
Wire Wire Line
	7200 1950 7400 1950
$Comp
L connectors:CONN_01X02 P104
U 1 1 591C2772
P 9600 5300
F 0 "P104" H 9678 5341 50  0000 L CNN
F 1 "JST" H 9678 5250 50  0000 L CNN
F 2 "th-normal:Connector_JST_PHR-2" H 9600 5300 60  0001 C CNN
F 3 "" H 9600 5300 60  0000 C CNN
	1    9600 5300
	1    0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR021
U 1 1 591C2864
P 9250 5450
F 0 "#PWR021" H 9250 5200 50  0001 C CNN
F 1 "GND" H 9255 5277 50  0000 C CNN
F 2 "" H 9250 5450 60  0000 C CNN
F 3 "" H 9250 5450 60  0000 C CNN
	1    9250 5450
	1    0    0    -1  
$EndComp
$Comp
L simple:R R111
U 1 1 591C28D8
P 9250 5050
F 0 "R111" H 9320 5096 50  0000 L CNN
F 1 "R" H 9320 5005 50  0000 L CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 9180 5050 30  0001 C CNN
F 3 "" H 9250 5050 30  0000 C CNN
	1    9250 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 5250 9400 5250
Wire Wire Line
	9250 5200 9250 5250
Connection ~ 9250 5250
Wire Wire Line
	9400 5350 9250 5350
Wire Wire Line
	9250 5350 9250 5450
Text Label 8750 5250 0    60   ~ 0
TAMPER
$Comp
L mspower:+3.3V #PWR022
U 1 1 591C2C8E
P 9250 4900
F 0 "#PWR022" H 9250 4750 50  0001 C CNN
F 1 "+3.3V" H 9265 5073 50  0000 C CNN
F 2 "" H 9250 4900 60  0000 C CNN
F 3 "" H 9250 4900 60  0000 C CNN
	1    9250 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 4250 4300 4250
Text Label 4350 4250 0    60   ~ 0
TAMPER
$Comp
L simple:R R106
U 1 1 591E0B44
P 7750 4450
F 0 "R106" H 7820 4496 50  0000 L CNN
F 1 "R" H 7820 4405 50  0000 L CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 7680 4450 30  0001 C CNN
F 3 "" H 7750 4450 30  0000 C CNN
	1    7750 4450
	1    0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR023
U 1 1 591E0BDB
P 7750 4600
F 0 "#PWR023" H 7750 4350 50  0001 C CNN
F 1 "GND" H 7755 4427 50  0000 C CNN
F 2 "" H 7750 4600 60  0000 C CNN
F 3 "" H 7750 4600 60  0000 C CNN
	1    7750 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 4250 7750 4300
Connection ~ 7750 4250
Text Notes 7700 4300 0    60   ~ 0
See MKE04P24M48SF0RM\n       Sect. 9.2 footnote 1
$Comp
L simple:C C107
U 1 1 591E14AD
P 3950 3600
F 0 "C107" V 3700 3550 50  0000 L CNN
F 1 "C" V 3800 3550 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 3988 3450 30  0001 C CNN
F 3 "" H 3950 3600 60  0000 C CNN
	1    3950 3600
	0    1    1    0   
$EndComp
$Comp
L simple:C C108
U 1 1 591E155E
P 3950 3900
F 0 "C108" V 4100 3850 50  0000 L CNN
F 1 "C" V 4200 3850 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 3988 3750 30  0001 C CNN
F 3 "" H 3950 3900 60  0000 C CNN
	1    3950 3900
	0    1    1    0   
$EndComp
$Comp
L mspower:GND #PWR024
U 1 1 591E1877
P 3200 3800
F 0 "#PWR024" H 3200 3550 50  0001 C CNN
F 1 "GND" H 3205 3627 50  0000 C CNN
F 2 "" H 3200 3800 60  0000 C CNN
F 3 "" H 3200 3800 60  0000 C CNN
	1    3200 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 3800 3200 3750
Wire Wire Line
	3200 3750 3400 3750
Wire Wire Line
	3400 3600 3400 3900
Wire Wire Line
	3400 3600 3800 3600
Wire Wire Line
	3400 3900 3800 3900
Connection ~ 3400 3750
$Comp
L templates:CRYSTAL X101
U 1 1 591E1AA3
P 4200 3750
F 0 "X101" V 4154 3833 50  0000 L CNN
F 1 "CRYSTAL" V 4400 3800 50  0000 L CNN
F 2 "th-normal:Crystal_HC49-U_Vertical" H 4200 3750 50  0001 C CNN
F 3 "" H 4200 3750 50  0000 C CNN
	1    4200 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	4100 3600 4500 3600
Wire Wire Line
	4200 3600 4200 3650
Wire Wire Line
	4100 3900 4500 3900
Wire Wire Line
	4200 3900 4200 3850
Wire Wire Line
	4500 3900 4500 3850
Wire Wire Line
	4500 3850 4850 3850
Connection ~ 4200 3900
Wire Wire Line
	4500 3600 4500 3750
Wire Wire Line
	4500 3750 4850 3750
Connection ~ 4200 3600
Wire Wire Line
	3950 4650 4850 4650
Text Label 4400 4650 0    60   ~ 0
CS
$Comp
L simple:C C109
U 1 1 591EAF00
P 4200 2700
F 0 "C109" H 4315 2746 50  0000 L CNN
F 1 "C" H 4315 2655 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 4238 2550 30  0001 C CNN
F 3 "" H 4200 2700 60  0000 C CNN
	1    4200 2700
	1    0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR025
U 1 1 591EB0C5
P 4200 2850
F 0 "#PWR025" H 4200 2600 50  0001 C CNN
F 1 "GND" H 4205 2677 50  0000 C CNN
F 2 "" H 4200 2850 60  0000 C CNN
F 3 "" H 4200 2850 60  0000 C CNN
	1    4200 2850
	1    0    0    -1  
$EndComp
$Comp
L mspower:+3.3V #PWR026
U 1 1 591EB10E
P 4200 2550
F 0 "#PWR026" H 4200 2400 50  0001 C CNN
F 1 "+3.3V" H 4215 2723 50  0000 C CNN
F 2 "" H 4200 2550 60  0000 C CNN
F 3 "" H 4200 2550 60  0000 C CNN
	1    4200 2550
	1    0    0    -1  
$EndComp
$Comp
L mspower:+3.3V #PWR027
U 1 1 591EB194
P 10050 3600
F 0 "#PWR027" H 10050 3450 50  0001 C CNN
F 1 "+3.3V" H 10065 3773 50  0000 C CNN
F 2 "" H 10050 3600 60  0000 C CNN
F 3 "" H 10050 3600 60  0000 C CNN
	1    10050 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 3550 6850 3550
Wire Wire Line
	7600 3450 6850 3450
Wire Wire Line
	6850 3250 7350 3250
Text Label 6900 3250 0    60   ~ 0
~RE_RESET
Text Label 10100 3750 2    60   ~ 0
~RE_RESET
$Comp
L mspower:GND #PWR028
U 1 1 591EB64E
P 9900 3850
F 0 "#PWR028" H 9900 3600 50  0001 C CNN
F 1 "GND" V 9905 3722 50  0000 R CNN
F 2 "" H 9900 3850 60  0000 C CNN
F 3 "" H 9900 3850 60  0000 C CNN
	1    9900 3850
	0    1    1    0   
$EndComp
Text Label 10100 3950 2    60   ~ 0
IRQ
Text Label 10100 4050 2    60   ~ 0
MISO
Text Label 10100 4150 2    60   ~ 0
MOSI
Text Label 10100 4250 2    60   ~ 0
SCK
Text Label 10100 4350 2    60   ~ 0
CS
Wire Wire Line
	10050 3600 10050 3650
Wire Wire Line
	10050 3650 10200 3650
Wire Wire Line
	9000 3750 10200 3750
Wire Wire Line
	10200 3850 9900 3850
Wire Wire Line
	10200 3950 9600 3950
Wire Wire Line
	10200 4050 9600 4050
Wire Wire Line
	10200 4150 9600 4150
Wire Wire Line
	10200 4250 9600 4250
Wire Wire Line
	10200 4350 9600 4350
$Comp
L simple:C C111
U 1 1 591EBE78
P 10500 2800
F 0 "C111" H 10615 2846 50  0000 L CNN
F 1 "C" H 10615 2755 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 10538 2650 30  0001 C CNN
F 3 "" H 10500 2800 60  0000 C CNN
	1    10500 2800
	1    0    0    -1  
$EndComp
$Comp
L mspower:+3.3V #PWR029
U 1 1 591EBF14
P 10500 2650
F 0 "#PWR029" H 10500 2500 50  0001 C CNN
F 1 "+3.3V" H 10515 2823 50  0000 C CNN
F 2 "" H 10500 2650 60  0000 C CNN
F 3 "" H 10500 2650 60  0000 C CNN
	1    10500 2650
	1    0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR030
U 1 1 591EBF5F
P 10500 2950
F 0 "#PWR030" H 10500 2700 50  0001 C CNN
F 1 "GND" H 10505 2777 50  0000 C CNN
F 2 "" H 10500 2950 60  0000 C CNN
F 3 "" H 10500 2950 60  0000 C CNN
	1    10500 2950
	1    0    0    -1  
$EndComp
$Comp
L capacitors:Jamicon_220uF_16V_TKP C110
U 1 1 591EC072
P 10250 2800
F 0 "C110" H 10364 2846 50  0000 L CNN
F 1 "Jamicon_220uF_16V_TKP" H 10364 2755 50  0000 L CNN
F 2 "th-normal:C_Radial_D6.3_L11.2_P2.5" H 10250 2800 60  0001 C CNN
F 3 "" H 10250 2800 60  0000 C CNN
	1    10250 2800
	-1   0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR031
U 1 1 591EC146
P 10250 2950
F 0 "#PWR031" H 10250 2700 50  0001 C CNN
F 1 "GND" H 10255 2777 50  0000 C CNN
F 2 "" H 10250 2950 60  0000 C CNN
F 3 "" H 10250 2950 60  0000 C CNN
	1    10250 2950
	1    0    0    -1  
$EndComp
$Comp
L mspower:+3.3V #PWR032
U 1 1 591EC193
P 10250 2650
F 0 "#PWR032" H 10250 2500 50  0001 C CNN
F 1 "+3.3V" H 10265 2823 50  0000 C CNN
F 2 "" H 10250 2650 60  0000 C CNN
F 3 "" H 10250 2650 60  0000 C CNN
	1    10250 2650
	1    0    0    -1  
$EndComp
$Comp
L capacitors:Jamicon_220uF_16V_TKP C105
U 1 1 591EC4FC
P 2650 5400
F 0 "C105" H 2765 5446 50  0000 L CNN
F 1 "Jamicon_220uF_16V_TKP" H 2765 5355 50  0000 L CNN
F 2 "th-normal:C_Radial_D6.3_L11.2_P2.5" H 2650 5400 60  0001 C CNN
F 3 "" H 2650 5400 60  0000 C CNN
	1    2650 5400
	1    0    0    -1  
$EndComp
$Comp
L simple:R R110
U 1 1 591ED0C5
P 9250 3950
F 0 "R110" H 9320 3996 50  0000 L CNN
F 1 "R" H 9320 3905 50  0000 L CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 9180 3950 30  0001 C CNN
F 3 "" H 9250 3950 30  0000 C CNN
	1    9250 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 3750 9250 3800
Connection ~ 9250 3750
$Comp
L mspower:GND #PWR033
U 1 1 591ED330
P 9250 4100
F 0 "#PWR033" H 9250 3850 50  0001 C CNN
F 1 "GND" H 9255 3927 50  0000 C CNN
F 2 "" H 9250 4100 60  0000 C CNN
F 3 "" H 9250 4100 60  0000 C CNN
	1    9250 4100
	1    0    0    -1  
$EndComp
$Comp
L simple:R R102
U 1 1 591ED3BF
P 3800 4650
F 0 "R102" V 3593 4650 50  0000 C CNN
F 1 "R" V 3684 4650 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 3730 4650 30  0001 C CNN
F 3 "" H 3800 4650 30  0000 C CNN
	1    3800 4650
	0    1    1    0   
$EndComp
$Comp
L mspower:+3.3V #PWR034
U 1 1 591ED52F
P 3500 4550
F 0 "#PWR034" H 3500 4400 50  0001 C CNN
F 1 "+3.3V" H 3515 4723 50  0000 C CNN
F 2 "" H 3500 4550 60  0000 C CNN
F 3 "" H 3500 4550 60  0000 C CNN
	1    3500 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 4550 3500 4650
Wire Wire Line
	3500 4650 3650 4650
$Comp
L mspower:VCC #PWR035
U 1 1 591EE20F
P 2850 6250
F 0 "#PWR035" H 2850 6100 50  0001 C CNN
F 1 "VCC" H 2867 6423 50  0000 C CNN
F 2 "" H 2850 6250 60  0000 C CNN
F 3 "" H 2850 6250 60  0000 C CNN
	1    2850 6250
	1    0    0    -1  
$EndComp
$Comp
L mspower:VCC #PWR036
U 1 1 591EE39C
P 2300 7150
F 0 "#PWR036" H 2300 7000 50  0001 C CNN
F 1 "VCC" H 2317 7323 50  0000 C CNN
F 2 "" H 2300 7150 60  0000 C CNN
F 3 "" H 2300 7150 60  0000 C CNN
	1    2300 7150
	1    0    0    -1  
$EndComp
$Comp
L mspower:PWR_FLAG #FLG037
U 1 1 591EE76F
P 750 4800
F 0 "#FLG037" H 750 4895 50  0001 C CNN
F 1 "PWR_FLAG" H 750 5023 50  0000 C CNN
F 2 "" H 750 4800 60  0000 C CNN
F 3 "" H 750 4800 60  0000 C CNN
	1    750  4800
	-1   0    0    1   
$EndComp
$Comp
L mspower:VCC #PWR038
U 1 1 591EE855
P 750 4800
F 0 "#PWR038" H 750 4650 50  0001 C CNN
F 1 "VCC" H 767 4973 50  0000 C CNN
F 2 "" H 750 4800 60  0000 C CNN
F 3 "" H 750 4800 60  0000 C CNN
	1    750  4800
	1    0    0    -1  
$EndComp
$Comp
L mspower:PWR_FLAG #FLG039
U 1 1 591EE8EE
P 1100 4850
F 0 "#FLG039" H 1100 4945 50  0001 C CNN
F 1 "PWR_FLAG" H 1100 5074 50  0000 C CNN
F 2 "" H 1100 4850 60  0000 C CNN
F 3 "" H 1100 4850 60  0000 C CNN
	1    1100 4850
	1    0    0    -1  
$EndComp
$Comp
L mspower:GND #PWR040
U 1 1 591EE98E
P 1100 4850
F 0 "#PWR040" H 1100 4600 50  0001 C CNN
F 1 "GND" H 1105 4677 50  0000 C CNN
F 2 "" H 1100 4850 60  0000 C CNN
F 3 "" H 1100 4850 60  0000 C CNN
	1    1100 4850
	1    0    0    -1  
$EndComp
NoConn ~ 4850 4150
Connection ~ 2500 2000
Text Label 2200 6400 0    60   ~ 0
485A
Text Label 2200 6700 0    60   ~ 0
485B
Wire Wire Line
	6850 3350 7350 3350
Text Label 6900 3350 0    60   ~ 0
IRQ
$Comp
L mspower:+3.3V #PWR041
U 1 1 59272043
P 7000 5000
F 0 "#PWR041" H 7000 4850 50  0001 C CNN
F 1 "+3.3V" V 7015 5128 50  0000 L CNN
F 2 "" H 7000 5000 60  0000 C CNN
F 3 "" H 7000 5000 60  0000 C CNN
	1    7000 5000
	0    -1   -1   0   
$EndComp
$Comp
L simple:R R112
U 1 1 5927213E
P 7200 5000
F 0 "R112" V 7300 5000 50  0000 C CNN
F 1 "R" V 7200 5000 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 7130 5000 30  0001 C CNN
F 3 "" H 7200 5000 30  0000 C CNN
	1    7200 5000
	0    -1   1    0   
$EndComp
Wire Wire Line
	7350 5000 7400 5000
Wire Wire Line
	7400 5000 7400 4650
Connection ~ 7400 4650
Wire Wire Line
	7000 5000 7050 5000
Text Notes 5200 6100 0    60   ~ 0
PTA3 is open drain output, external pull-up might be\nnecessary see Figure 11-2 in the Kinetis KEO4Z\nuser manual.\n\nBut the input of SN75176 is pulled up internally so\nin this case the resistor is probably not needed. \nSee Figure 19 of TI's sn75176b.pdf\n\nI am keeping the R here just in case it is needed\nfor reliable operation.
$Comp
L simple:R R?
U 1 1 5A10499B
P 3550 3750
F 0 "R?" H 3620 3796 50  0000 L CNN
F 1 "10M" H 3620 3705 50  0000 L CNN
F 2 "" V 3480 3750 30  0000 C CNN
F 3 "" H 3550 3750 30  0000 C CNN
	1    3550 3750
	1    0    0    -1  
$EndComp
Connection ~ 3550 3600
Connection ~ 3550 3900
$EndSCHEMATC
