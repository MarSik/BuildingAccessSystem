EESchema Schematic File Version 2
LIBS:74xx
LIBS:boards
LIBS:buttons
LIBS:capacitors
LIBS:connectors
LIBS:cpu
LIBS:discrete
LIBS:drivers
LIBS:ic
LIBS:mspower
LIBS:opto
LIBS:passives
LIBS:power
LIBS:relay
LIBS:rf
LIBS:simple
LIBS:transistors
LIBS:templates
LIBS:dvernik-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
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
L Mini-360 U401
U 1 1 590E228F
P 5000 1600
F 0 "U401" H 5000 2087 60  0000 C CNN
F 1 "Mini-360" H 5000 1981 60  0000 C CNN
F 2 "th-normal:DcDc_MH_MINI_360" H 5000 1600 60  0001 C CNN
F 3 "" H 5000 1600 60  0000 C CNN
	1    5000 1600
	1    0    0    -1  
$EndComp
$Comp
L C C404
U 1 1 590E22AE
P 6300 1600
F 0 "C404" H 6415 1646 50  0000 L CNN
F 1 "C" H 6415 1555 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 6338 1450 30  0001 C CNN
F 3 "" H 6300 1600 60  0000 C CNN
	1    6300 1600
	1    0    0    -1  
$EndComp
$Comp
L FUSE F401
U 1 1 590E22B5
P 1800 2050
F 0 "F401" V 1500 2050 50  0000 C CNN
F 1 "FUSE" V 1600 2050 50  0000 C CNN
F 2 "th-normal:Fuse_5x20_Schurter-0031-8201" V 1730 2050 30  0001 C CNN
F 3 "" H 1800 2050 30  0000 C CNN
	1    1800 2050
	0    1    1    0   
$EndComp
$Comp
L IRFU9024 Q401
U 1 1 590E22BC
P 2650 2150
F 0 "Q401" V 3050 2150 50  0000 C CNN
F 1 "IRFU9024" V 2950 2150 50  0000 C CNN
F 2 "smd-handsolder:DPAK_HandSoldering" H 2850 2250 29  0001 C CNN
F 3 "" H 2650 2150 60  0000 C CNN
	1    2650 2150
	0    1    -1   0   
$EndComp
$Comp
L CONN_01X02 P401
U 1 1 590E22C3
P 1100 2250
F 0 "P401" H 1250 1950 50  0000 R CNN
F 1 "WAGO255" H 1250 2050 50  0000 R CNN
F 2 "th-npth:Wire-2_WAGO-255_P5mm_HandSoldering" H 1100 2250 60  0001 C CNN
F 3 "" H 1100 2250 60  0000 C CNN
	1    1100 2250
	-1   0    0    1   
$EndComp
Text Notes 1050 2700 0    60   ~ 0
Main power input - 12V\npolarity and overvoltage protected
$Comp
L Jamicon_220uF_16V_TKP C402
U 1 1 590E22CB
P 5650 1600
F 0 "C402" H 5765 1646 50  0000 L CNN
F 1 "Jamicon_220uF_16V_TKP" H 5750 1300 50  0000 L CNN
F 2 "th-normal:C_Radial_D6.3_L11.2_P2.5" H 5650 1600 60  0001 C CNN
F 3 "" H 5650 1600 60  0000 C CNN
	1    5650 1600
	1    0    0    -1  
$EndComp
$Comp
L C C405
U 1 1 590E2568
P 6300 2800
F 0 "C405" H 6415 2846 50  0000 L CNN
F 1 "C" H 6415 2755 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 6338 2650 30  0001 C CNN
F 3 "" H 6300 2800 60  0000 C CNN
	1    6300 2800
	1    0    0    -1  
$EndComp
Text HLabel 7000 1200 2    60   UnSpc ~ 0
+5V
Text HLabel 7000 2600 2    60   UnSpc ~ 0
+12V
Text HLabel 7000 3450 2    60   UnSpc ~ 0
GND
Text Notes 4950 1650 0    60   ~ 0
5V
Text Notes 4950 2850 0    60   ~ 0
12V
$Comp
L BZX85C15 D401
U 1 1 590E6F05
P 2050 2250
F 0 "D401" V 2004 2329 50  0000 L CNN
F 1 "BZX85C15" V 2095 2329 50  0000 L CNN
F 2 "th-normal:Diode_DO-41_SOD81_Horizontal_RM10" H 2050 2250 60  0001 C CNN
F 3 "" H 2050 2250 60  0000 C CNN
	1    2050 2250
	0    1    1    0   
$EndComp
$Comp
L Jamicon_220uF_16V_TKP C403
U 1 1 590E256E
P 5650 2800
F 0 "C403" H 5765 2846 50  0000 L CNN
F 1 "Jamicon_220uF_16V_TKP" H 5750 2500 50  0000 L CNN
F 2 "th-normal:C_Radial_D6.3_L11.2_P2.5" H 5650 2800 60  0001 C CNN
F 3 "" H 5650 2800 60  0000 C CNN
	1    5650 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 1400 6300 1400
Wire Wire Line
	5650 1200 5650 1450
Wire Wire Line
	5400 1800 6300 1800
Wire Wire Line
	5650 1750 5650 2100
Wire Wire Line
	2850 2050 4200 2050
Connection ~ 5650 1400
Connection ~ 5650 1800
Wire Wire Line
	6300 1400 6300 1450
Wire Wire Line
	6300 1800 6300 1750
Wire Wire Line
	1950 2050 2450 2050
Wire Wire Line
	2850 1950 2950 1950
Wire Wire Line
	2950 1950 2950 2050
Connection ~ 2950 2050
Wire Wire Line
	2650 2450 2650 2350
Wire Wire Line
	1300 2200 1500 2200
Wire Wire Line
	1300 2300 1500 2300
Wire Wire Line
	1500 2300 1500 2450
Connection ~ 2650 2450
Wire Wire Line
	1500 2050 1650 2050
Wire Wire Line
	1500 2200 1500 2050
Wire Wire Line
	4200 1400 4200 2600
Wire Wire Line
	4200 1400 4600 1400
Wire Wire Line
	4200 2600 7000 2600
Connection ~ 4200 2050
Wire Wire Line
	4450 1800 4450 3450
Wire Wire Line
	4450 1800 4600 1800
Wire Wire Line
	5650 3450 5650 2950
Wire Wire Line
	6300 2600 6300 2650
Wire Wire Line
	6300 3450 6300 2950
Wire Wire Line
	7000 1200 5650 1200
Wire Wire Line
	3050 3450 7000 3450
Connection ~ 5650 3450
Wire Wire Line
	5650 2100 6750 2100
Wire Wire Line
	6750 2100 6750 3450
Connection ~ 6750 3450
Connection ~ 2050 2450
Connection ~ 2050 2050
Wire Wire Line
	3050 2450 3050 3450
Wire Wire Line
	1500 2450 3050 2450
Connection ~ 4450 3450
Connection ~ 6300 3450
Wire Wire Line
	5650 2650 5650 2600
Connection ~ 5650 2600
Connection ~ 6300 2600
Text Label 1450 2200 2    60   ~ 0
PIN
Text Label 2050 2050 0    60   ~ 0
PFUSED
$EndSCHEMATC
