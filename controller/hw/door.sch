EESchema Schematic File Version 4
LIBS:dvernik-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
Title ""
Date "2017-11-28"
Rev "1.2"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L relay:JQX-14FC22CS8-5.0 U201
U 1 1 58FE8C56
P 5700 3300
AR Path="/58FE8759/58FE8C56" Ref="U201"  Part="1" 
AR Path="/58FEA049/58FE8C56" Ref="U301"  Part="1" 
F 0 "U201" V 5550 2650 60  0000 R CNN
F 1 "JQX-14FC22CS8-5.0" V 5450 2650 60  0000 R CNN
F 2 "th-npth:Relay_JQX-14FC_L5_HandSoldering" H 5700 3350 60  0001 C CNN
F 3 "" H 5700 3350 60  0000 C CNN
	1    5700 3300
	0    -1   -1   0   
$EndComp
$Comp
L transistors:2N7002K Q201
U 1 1 58FE8C63
P 5300 4100
AR Path="/58FE8759/58FE8C63" Ref="Q201"  Part="1" 
AR Path="/58FEA049/58FE8C63" Ref="Q301"  Part="1" 
F 0 "Q201" H 5491 4146 50  0000 L CNN
F 1 "2N7002K" H 5491 4055 50  0000 L CNN
F 2 "smd-handsolder:SOT-23_HandSoldering" H 5500 4200 29  0001 C CNN
F 3 "" H 5300 4100 60  0000 C CNN
	1    5300 4100
	1    0    0    -1  
$EndComp
$Comp
L simple:SK110 D201
U 1 1 58FE8C6A
P 4900 3300
AR Path="/58FE8759/58FE8C6A" Ref="D201"  Part="1" 
AR Path="/58FEA049/58FE8C6A" Ref="D301"  Part="1" 
F 0 "D201" V 4854 3379 50  0000 L CNN
F 1 "SK110" V 4945 3379 50  0000 L CNN
F 2 "smd-handsolder:Diode-SMA_Handsoldering" H 4900 3300 60  0001 C CNN
F 3 "" H 4900 3300 60  0000 C CNN
	1    4900 3300
	0    1    1    0   
$EndComp
$Comp
L simple:R R201
U 1 1 58FE8C83
P 4650 4300
AR Path="/58FE8759/58FE8C83" Ref="R201"  Part="1" 
AR Path="/58FEA049/58FE8C83" Ref="R301"  Part="1" 
F 0 "R201" H 4720 4346 50  0000 L CNN
F 1 "R" H 4720 4255 50  0000 L CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 4580 4300 30  0001 C CNN
F 3 "" H 4650 4300 30  0000 C CNN
	1    4650 4300
	1    0    0    -1  
$EndComp
Text Notes 3150 5200 0    60   ~ 0
TODO - is the mosfet strong enough?\n24mA at Vgs = 3.3V; Vds = 5V within specs\n(Rdson cca 5 ohm)
$Comp
L capacitors:Jamicon_220uF_16V_TKP C201
U 1 1 58FE8C97
P 4350 3000
AR Path="/58FE8759/58FE8C97" Ref="C201"  Part="1" 
AR Path="/58FEA049/58FE8C97" Ref="C301"  Part="1" 
F 0 "C201" H 4465 3046 50  0000 L CNN
F 1 "Jamicon_220uF_16V_TKP" H 4465 2955 50  0000 L CNN
F 2 "th-normal:C_Radial_D6.3_L11.2_P2.5" H 4350 3000 60  0001 C CNN
F 3 "" H 4350 3000 60  0000 C CNN
	1    4350 3000
	1    0    0    -1  
$EndComp
Text HLabel 4150 4100 0    60   Input ~ 0
OPEN
$Comp
L connectors:CONN_01X02 P201
U 1 1 58FE8E7A
P 6350 2300
AR Path="/58FE8759/58FE8E7A" Ref="P201"  Part="1" 
AR Path="/58FEA049/58FE8E7A" Ref="P301"  Part="1" 
F 0 "P201" H 6428 2341 50  0000 L CNN
F 1 "WAGO255" H 6428 2250 50  0000 L CNN
F 2 "th-npth:Wire-2_WAGO-255_P5mm_HandSoldering" H 6350 2300 60  0001 C CNN
F 3 "" H 6350 2300 60  0000 C CNN
	1    6350 2300
	-1   0    0    1   
$EndComp
Text HLabel 6800 5350 0    60   UnSpc ~ 0
BUZZER_PWR_1
Text HLabel 6800 5450 0    60   UnSpc ~ 0
BUZZER_PWR_2
Text Label 6600 2350 0    60   ~ 0
BUZZER_IN1
Text Label 6600 2250 0    60   ~ 0
BUZZER_IN2
Text Label 7700 4100 0    60   ~ 0
BUZZER_NO
Text Label 7700 4200 0    60   ~ 0
BUZZER_COM
Text HLabel 4150 4700 0    60   UnSpc ~ 0
LOGIC_GND
Text HLabel 4150 2550 0    60   UnSpc ~ 0
LOGIC_5V
Text Notes 6300 1600 0    60   ~ 0
Cut the existing buzzer wires and connect\nthe control side here, polarity should not matter.
Text Notes 7250 4650 0    60   ~ 0
Connect the buzzer side of cut buzzer wires here\n(connect to COM+NO or COM+NC depending on the door)
Text Notes 2950 4000 0    60   ~ 0
Log. high triggers the relay\nand opens the door
$Comp
L connectors:CONN_01X03 P202
U 1 1 58FEE200
P 8600 4200
AR Path="/58FE8759/58FEE200" Ref="P202"  Part="1" 
AR Path="/58FEA049/58FEE200" Ref="P302"  Part="1" 
F 0 "P202" H 8678 4241 50  0000 L CNN
F 1 "WAGO255" H 8678 4150 50  0000 L CNN
F 2 "th-npth:Wire-3_WAGO-255_P5mm_HandSoldering" H 8600 4200 60  0001 C CNN
F 3 "" H 8600 4200 60  0000 C CNN
	1    8600 4200
	1    0    0    1   
$EndComp
Text Label 7700 4300 0    60   ~ 0
BUZZER_NC
Text Notes 5800 5750 0    60   ~ 0
The replacement buzzer power,\npolarity normally does not matter
$Comp
L simple:SD103A D205
U 1 1 5916DE27
P 8000 2600
F 0 "D205" V 7954 2679 50  0000 L CNN
F 1 "SD103A" V 8045 2679 50  0000 L CNN
F 2 "th-npth:Diode_DO-35_SOD27_Horizontal_RM10" H 8000 2600 60  0001 C CNN
F 3 "" H 8000 2600 60  0000 C CNN
	1    8000 2600
	0    1    1    0   
$EndComp
$Comp
L simple:SD103A D202
U 1 1 5916DE99
P 7500 1950
F 0 "D202" V 7546 2029 50  0000 L CNN
F 1 "SD103A" V 7455 2029 50  0000 L CNN
F 2 "th-npth:Diode_DO-35_SOD27_Horizontal_RM10" H 7500 1950 60  0001 C CNN
F 3 "" H 7500 1950 60  0000 C CNN
	1    7500 1950
	0    1    -1   0   
$EndComp
$Comp
L simple:SD103A D204
U 1 1 5916DEC5
P 8000 1950
F 0 "D204" V 7954 2029 50  0000 L CNN
F 1 "SD103A" V 8045 2029 50  0000 L CNN
F 2 "th-npth:Diode_DO-35_SOD27_Horizontal_RM10" H 8000 1950 60  0001 C CNN
F 3 "" H 8000 1950 60  0000 C CNN
	1    8000 1950
	0    1    1    0   
$EndComp
$Comp
L simple:SD103A D203
U 1 1 5916DEF3
P 7500 2600
F 0 "D203" V 7546 2521 50  0000 R CNN
F 1 "SD103A" V 7455 2521 50  0000 R CNN
F 2 "th-npth:Diode_DO-35_SOD27_Horizontal_RM10" H 7500 2600 60  0001 C CNN
F 3 "" H 7500 2600 60  0000 C CNN
	1    7500 2600
	0    -1   -1   0   
$EndComp
$Comp
L opto:Cosmo_1010-817 U202
U 1 1 5916DFF3
P 9800 3100
F 0 "U202" H 9800 3425 50  0000 C CNN
F 1 "Cosmo_1010-817" H 9800 3334 50  0000 C CNN
F 2 "th-normal:DIP-4_W300mil" H 9600 2900 50  0001 L CIN
F 3 "" H 9800 3000 50  0000 L CNN
	1    9800 3100
	1    0    0    -1  
$EndComp
$Comp
L simple:R R202
U 1 1 5916E13C
P 9250 3200
F 0 "R202" V 9150 3200 50  0000 C CNN
F 1 "R" V 9250 3200 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 9180 3200 30  0001 C CNN
F 3 "" H 9250 3200 30  0000 C CNN
	1    9250 3200
	0    1    1    0   
$EndComp
Text HLabel 10250 3200 2    60   UnSpc ~ 0
SENS_EM
Text HLabel 10250 3000 2    60   UnSpc ~ 0
SENS_CO
Wire Wire Line
	10100 3200 10250 3200
Wire Wire Line
	10100 3000 10250 3000
Connection ~ 8000 3200
Wire Wire Line
	7500 1700 7500 1800
Wire Wire Line
	7250 1700 7500 1700
Wire Wire Line
	7250 3200 7250 1700
Connection ~ 7500 2250
Wire Wire Line
	7500 2250 7500 2100
Connection ~ 8450 3000
Wire Wire Line
	8450 1700 8450 3000
Wire Wire Line
	8000 1700 8450 1700
Wire Wire Line
	8000 1800 8000 1700
Wire Wire Line
	8000 2250 8000 2100
Wire Wire Line
	6550 2250 8000 2250
Wire Wire Line
	8000 3200 8000 2750
Wire Wire Line
	7250 3200 9100 3200
Wire Wire Line
	7500 2750 7500 3000
Connection ~ 7500 2350
Wire Wire Line
	8000 2350 8000 2450
Wire Wire Line
	7500 2350 7500 2450
Wire Wire Line
	6550 2350 8000 2350
Wire Wire Line
	9400 3200 9500 3200
Wire Wire Line
	7500 3000 9500 3000
Wire Wire Line
	6950 4200 6950 5350
Wire Wire Line
	5800 4300 5800 3700
Wire Wire Line
	8400 4300 5800 4300
Wire Wire Line
	5400 2550 5400 2900
Wire Wire Line
	4350 2800 5400 2800
Wire Wire Line
	4900 2800 4900 3150
Connection ~ 5400 2800
Wire Wire Line
	4900 3450 4900 3800
Wire Wire Line
	4900 3800 5400 3800
Wire Wire Line
	5400 3700 5400 3900
Connection ~ 5400 3800
Wire Wire Line
	5400 4700 5400 4300
Wire Wire Line
	4150 4100 5100 4100
Wire Wire Line
	4650 4100 4650 4150
Connection ~ 4650 4100
Wire Wire Line
	4650 4450 4650 4700
Wire Wire Line
	4350 2800 4350 2850
Connection ~ 4900 2800
Wire Wire Line
	6950 5350 6800 5350
Wire Wire Line
	6950 4200 5900 4200
Wire Wire Line
	5900 4200 5900 3700
Wire Wire Line
	6000 3700 6000 4100
Wire Wire Line
	6000 4100 8400 4100
Wire Wire Line
	6800 5450 7050 5450
Wire Wire Line
	7050 4200 8400 4200
Wire Wire Line
	4150 4700 5400 4700
Connection ~ 4650 4700
Wire Wire Line
	4350 3150 4350 4700
Connection ~ 4350 4700
Wire Wire Line
	4150 2550 5400 2550
$Comp
L templates:FUSE F201
U 1 1 591740C4
P 7050 5050
F 0 "F201" H 7120 5096 50  0000 L CNN
F 1 "FUSE" H 7120 5005 50  0000 L CNN
F 2 "th-normal:Fuse_5x20_Schurter-0031-8201" V 6980 5050 30  0001 C CNN
F 3 "" H 7050 5050 30  0000 C CNN
	1    7050 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 5450 7050 5200
Wire Wire Line
	7050 4900 7050 4200
Wire Wire Line
	5800 2900 5800 2800
Wire Wire Line
	5800 2800 5700 2800
Wire Wire Line
	5700 2800 5700 3750
Wire Wire Line
	5700 3750 5800 3750
Connection ~ 5800 3750
Wire Wire Line
	6000 3750 6150 3750
Wire Wire Line
	6150 3750 6150 2800
Wire Wire Line
	6150 2800 6000 2800
Wire Wire Line
	6000 2800 6000 2900
Connection ~ 6000 3750
Wire Wire Line
	5900 2900 5900 2700
Wire Wire Line
	5900 2700 6250 2700
Wire Wire Line
	6250 2700 6250 3850
Wire Wire Line
	6250 3850 5900 3850
Connection ~ 5900 3850
Text Label 5000 3800 0    60   ~ 0
RELAYGND
$EndSCHEMATC
