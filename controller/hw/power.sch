EESchema Schematic File Version 4
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
L power:Mini-360 U401
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
L simple:C C404
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
L templates:FUSE F401
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
L transistors:IRFU9024 Q401
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
L connectors:CONN_01X02 P401
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
Main power input - cca 13.8V\npolarity and overvoltage protected
$Comp
L capacitors:Jamicon_220uF_16V_TKP C402
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
L simple:C C405
U 1 1 590E2568
P 6300 2800
F 0 "C405" H 6415 2846 50  0000 L CNN
F 1 "C" H 6415 2755 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 6338 2650 30  0001 C CNN
F 3 "" H 6300 2800 60  0000 C CNN
	1    6300 2800
	1    0    0    -1  
$EndComp
Text HLabel 10250 1200 2    60   UnSpc ~ 0
+5V
Text HLabel 7000 2600 2    60   UnSpc ~ 0
+12V
Text HLabel 10250 3450 2    60   UnSpc ~ 0
GND
Text Notes 4950 1650 0    60   ~ 0
5V
Text Notes 4950 2850 0    60   ~ 0
12V
$Comp
L discrete:BZX85C15 D401
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
L capacitors:Jamicon_220uF_16V_TKP C403
U 1 1 590E256E
P 5650 2800
F 0 "C403" H 5765 2846 50  0000 L CNN
F 1 "Jamicon_220uF_16V_TKP" H 5750 2500 50  0000 L CNN
F 2 "th-normal:C_Radial_D6.3_L11.2_P2.5" H 5650 2800 60  0001 C CNN
F 3 "" H 5650 2800 60  0000 C CNN
	1    5650 2800
	1    0    0    -1  
$EndComp
Text Label 1450 2200 2    60   ~ 0
PIN
Text Label 2050 2050 0    60   ~ 0
PFUSED
$Comp
L power:Mini-360 U402
U 1 1 5922A582
P 5000 2800
F 0 "U402" H 5000 3287 60  0000 C CNN
F 1 "Mini-360" H 5000 3181 60  0000 C CNN
F 2 "th-normal:DcDc_MH_MINI_360" H 5000 2800 60  0001 C CNN
F 3 "" H 5000 2800 60  0000 C CNN
	1    5000 2800
	1    0    0    -1  
$EndComp
$Comp
L capacitors:Jamicon_220uF_25V C401
U 1 1 5922BD9A
P 3250 2300
F 0 "C401" H 3365 2346 50  0000 L CNN
F 1 "Jamicon_220uF_25V" H 3365 2255 50  0000 L CNN
F 2 "th-normal:C_Radial_D8_L11.5_P3.5" H 3250 2300 60  0001 C CNN
F 3 "" H 3250 2300 60  0000 C CNN
	1    3250 2300
	1    0    0    -1  
$EndComp
$Comp
L simple:C C412
U 1 1 5922BEB6
P 3600 2750
F 0 "C412" H 3715 2796 50  0000 L CNN
F 1 "C" H 3715 2705 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 3638 2600 30  0001 C CNN
F 3 "" H 3600 2750 60  0000 C CNN
	1    3600 2750
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
Connection ~ 4200 2050
Wire Wire Line
	4450 3450 4450 1800
Wire Wire Line
	4450 1800 4600 1800
Wire Wire Line
	5650 3450 5650 2950
Wire Wire Line
	6300 2600 6300 2650
Wire Wire Line
	6300 3450 6300 2950
Wire Wire Line
	10250 1200 5650 1200
Wire Wire Line
	3050 3450 10250 3450
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
Wire Wire Line
	4200 2600 4600 2600
Wire Wire Line
	5400 2600 7000 2600
Wire Wire Line
	4600 3000 4500 3000
Wire Wire Line
	4500 3000 4500 3450
Connection ~ 4500 3450
Wire Wire Line
	5400 3000 5500 3000
Wire Wire Line
	5500 3000 5500 3450
Connection ~ 5500 3450
Wire Wire Line
	3250 2150 3250 2050
Connection ~ 3250 2050
Wire Wire Line
	3250 2450 3250 3450
Connection ~ 3250 3450
Wire Wire Line
	3600 2600 3600 2050
Connection ~ 3600 2050
Wire Wire Line
	3600 2900 3600 3450
Connection ~ 3600 3450
$Comp
L simple:TL431ACD U408
U 1 1 5925D1B0
P 8700 2250
F 0 "U408" H 9088 2303 60  0000 L CNN
F 1 "TL431ACD" H 9088 2197 60  0000 L CNN
F 2 "" H 9550 2100 60  0001 C CNN
F 3 "" H 9550 2100 60  0001 C CNN
	1    8700 2250
	1    0    0    -1  
$EndComp
$Comp
L simple:R R412
U 1 1 5925D21E
P 8000 1950
F 0 "R412" H 8070 1996 50  0000 L CNN
F 1 "12k" H 8070 1905 50  0000 L CNN
F 2 "" V 7930 1950 30  0001 C CNN
F 3 "" H 8000 1950 30  0000 C CNN
	1    8000 1950
	1    0    0    -1  
$EndComp
$Comp
L simple:R R413
U 1 1 5925D25E
P 8000 2350
F 0 "R413" H 8070 2396 50  0000 L CNN
F 1 "10k" H 8070 2305 50  0000 L CNN
F 2 "" V 7930 2350 30  0001 C CNN
F 3 "" H 8000 2350 30  0000 C CNN
	1    8000 2350
	1    0    0    -1  
$EndComp
$Comp
L simple:R R414
U 1 1 5925D32B
P 8900 1600
F 0 "R414" H 8970 1646 50  0000 L CNN
F 1 "R" H 8970 1555 50  0000 L CNN
F 2 "" V 8830 1600 30  0001 C CNN
F 3 "" H 8900 1600 30  0000 C CNN
	1    8900 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 2150 8000 2150
Wire Wire Line
	8000 2100 8000 2200
Connection ~ 8000 2150
Wire Wire Line
	8000 2500 8000 2800
Wire Wire Line
	8000 2800 9350 2800
Wire Wire Line
	9000 2800 9000 2650
Wire Wire Line
	8550 2800 8550 3450
Connection ~ 8550 3450
Connection ~ 8550 2800
Wire Wire Line
	8700 2650 8700 2800
Connection ~ 8700 2800
Wire Wire Line
	8800 2650 8800 2800
Connection ~ 8800 2800
Wire Wire Line
	8900 2650 8900 2800
Connection ~ 8900 2800
Wire Wire Line
	8000 1800 8000 1350
Wire Wire Line
	8000 1350 9850 1350
Wire Wire Line
	8900 1450 8900 1350
Connection ~ 8900 1350
Wire Wire Line
	8900 1750 8900 1850
Wire Wire Line
	8900 1800 9550 1800
Connection ~ 8900 1800
Wire Wire Line
	8550 1350 8550 1200
Connection ~ 8550 1200
Connection ~ 8550 1350
Text Notes 7900 1100 0    60   ~ 0
Overvoltage crowbar protection
$Comp
L simple:C C413
U 1 1 5925DDF5
P 9350 2600
F 0 "C413" H 9465 2646 50  0000 L CNN
F 1 "C" H 9465 2555 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 9388 2450 30  0001 C CNN
F 3 "" H 9350 2600 60  0000 C CNN
	1    9350 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 2450 9350 1800
Connection ~ 9350 1800
Wire Wire Line
	9350 2800 9350 2750
Connection ~ 9000 2800
Text Notes 9900 2350 0    60   ~ 0
Triak, MT1 nahoře\nnapř. Z0103MA 5AL2
$EndSCHEMATC
