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
Sheet 3 4
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
L ROE-0505S U403
U 1 1 5917718B
P 5300 1300
F 0 "U403" H 5650 1750 60  0000 L CNN
F 1 "ROE-0505S" H 5650 1650 60  0000 L CNN
F 2 "th-normal:DcDc_Roe_SIP-4" H 5300 1300 60  0001 C CNN
F 3 "" H 5300 1300 60  0000 C CNN
	1    5300 1300
	1    0    0    -1  
$EndComp
$Comp
L R R402
U 1 1 591771B1
P 4150 2000
F 0 "R402" V 4250 1900 50  0000 L CNN
F 1 "1k" V 4150 1950 50  0000 L CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 4080 2000 30  0001 C CNN
F 3 "" H 4150 2000 30  0000 C CNN
	1    4150 2000
	0    1    1    0   
$EndComp
$Comp
L R R406
U 1 1 591771B8
P 6350 2100
F 0 "R406" H 6420 2146 50  0000 L CNN
F 1 "1k" H 6420 2055 50  0000 L CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 6280 2100 30  0001 C CNN
F 3 "" H 6350 2100 30  0000 C CNN
	1    6350 2100
	1    0    0    -1  
$EndComp
$Comp
L R R405
U 1 1 591771C6
P 5900 2600
F 0 "R405" V 5800 2600 50  0000 C CNN
F 1 "1k" V 5900 2600 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 5830 2600 30  0001 C CNN
F 3 "" H 5900 2600 30  0000 C CNN
	1    5900 2600
	0    1    1    0   
$EndComp
$Comp
L R R404
U 1 1 591771CD
P 4700 3250
F 0 "R404" V 4600 3250 50  0000 C CNN
F 1 "1k" V 4700 3250 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 4630 3250 30  0001 C CNN
F 3 "" H 4700 3250 30  0000 C CNN
	1    4700 3250
	0    1    1    0   
$EndComp
$Comp
L GND #PWR301
U 1 1 591771F0
P 5000 2400
F 0 "#PWR301" H 5000 2150 50  0001 C CNN
F 1 "GND" H 5005 2227 50  0000 C CNN
F 2 "" H 5000 2400 60  0000 C CNN
F 3 "" H 5000 2400 60  0000 C CNN
	1    5000 2400
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR303
U 1 1 5917720A
P 5850 3250
F 0 "#PWR303" H 5850 3000 50  0001 C CNN
F 1 "GNDA" H 5855 3077 50  0000 C CNN
F 2 "" H 5850 3250 60  0000 C CNN
F 3 "" H 5850 3250 60  0000 C CNN
	1    5850 3250
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR302
U 1 1 5917721C
P 5800 1550
F 0 "#PWR302" H 5800 1300 50  0001 C CNN
F 1 "GNDA" H 5805 1377 50  0000 C CNN
F 2 "" H 5800 1550 60  0000 C CNN
F 3 "" H 5800 1550 60  0000 C CNN
	1    5800 1550
	1    0    0    -1  
$EndComp
Text Notes 6600 1000 0    60   ~ 0
fully isolated to protect safe\nzone from unsafe zone
Text Label 4400 2500 2    60   ~ 0
CPU_RX
Text Label 4400 3250 2    60   ~ 0
CPU_TX
$Comp
L +5VA #PWR304
U 1 1 5917722F
P 6350 1100
F 0 "#PWR304" H 6350 950 50  0001 C CNN
F 1 "+5VA" H 6365 1273 50  0000 C CNN
F 2 "" H 6350 1100 60  0000 C CNN
F 3 "" H 6350 1100 60  0000 C CNN
	1    6350 1100
	1    0    0    -1  
$EndComp
$Comp
L C C407
U 1 1 5917723B
P 4650 1300
F 0 "C407" H 4700 1400 50  0000 L CNN
F 1 "100n" H 4700 1200 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 4688 1150 30  0001 C CNN
F 3 "" H 4650 1300 60  0000 C CNN
	1    4650 1300
	1    0    0    -1  
$EndComp
$Comp
L C C408
U 1 1 59177249
P 5800 1300
F 0 "C408" H 5850 1400 50  0000 L CNN
F 1 "100n" H 5850 1200 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 5838 1150 30  0001 C CNN
F 3 "" H 5800 1300 60  0000 C CNN
	1    5800 1300
	1    0    0    -1  
$EndComp
Text Label 6500 2600 0    60   ~ 0
UNSAFE_RX
Text Label 6500 3050 0    60   ~ 0
UNSAFE_TX
$Comp
L HCPL-0601-500E U404
U 1 1 5917725E
P 5300 2250
F 0 "U404" H 5600 2800 50  0000 C CNN
F 1 "HCPL-0601-500E" H 5300 2700 50  0000 C CNN
F 2 "smd-normal:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 5300 1600 50  0001 L CIN
F 3 "" H 5300 2150 50  0000 L CNN
	1    5300 2250
	-1   0    0    -1  
$EndComp
$Comp
L HCPL-0601-500E U405
U 1 1 59177265
P 5300 3150
F 0 "U405" H 5400 3700 50  0000 C CNN
F 1 "HCPL-0601-500E" H 5300 3600 50  0000 C CNN
F 2 "smd-normal:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 5300 2500 50  0001 L CIN
F 3 "" H 5300 3050 50  0000 L CNN
	1    5300 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 2250 6350 3050
Wire Wire Line
	5600 3050 7550 3050
Wire Wire Line
	6350 1100 6350 1950
Connection ~ 6350 1100
Wire Wire Line
	3450 1100 4900 1100
Wire Wire Line
	3600 2500 4750 2500
Wire Wire Line
	5600 2150 6150 2150
Connection ~ 6150 1100
Wire Wire Line
	3900 3050 5000 3050
Wire Wire Line
	4850 3250 5000 3250
Wire Wire Line
	3600 3250 4550 3250
Wire Wire Line
	6050 2600 7350 2600
Wire Wire Line
	5600 3250 5850 3250
Connection ~ 6350 3050
Connection ~ 6150 2150
Wire Wire Line
	5800 1450 5800 1550
Connection ~ 5800 1500
Wire Wire Line
	1900 1500 4900 1500
Wire Wire Line
	2300 1150 2300 1100
Connection ~ 2300 1100
Wire Wire Line
	4650 1100 4650 1150
Connection ~ 4650 1100
Wire Wire Line
	2300 1450 2300 1500
Connection ~ 2300 1500
Wire Wire Line
	4650 1500 4650 1450
Connection ~ 4650 1500
Wire Wire Line
	5800 1150 5800 1100
Connection ~ 5800 1100
Wire Wire Line
	5600 2350 5750 2350
Wire Wire Line
	5650 2350 5650 2600
Wire Wire Line
	5650 2600 5750 2600
Wire Wire Line
	5000 2350 5000 2400
Wire Wire Line
	6150 2900 5600 2900
Wire Wire Line
	5000 2900 4900 2900
Wire Wire Line
	4900 2900 4900 2750
Wire Wire Line
	4900 2750 5700 2750
Wire Wire Line
	5700 2750 5700 2900
Connection ~ 5700 2900
Wire Wire Line
	5600 2000 5700 2000
Wire Wire Line
	5700 2000 5700 1850
Wire Wire Line
	5700 1850 4900 1850
Wire Wire Line
	4900 1850 4900 2000
Wire Wire Line
	4900 2000 5000 2000
Wire Wire Line
	3900 2000 4000 2000
Wire Wire Line
	3900 1100 3900 4650
Connection ~ 3900 1900
Wire Wire Line
	4300 2000 4450 2000
Wire Wire Line
	4450 2000 4450 2500
Connection ~ 4450 2500
Wire Wire Line
	4750 2500 4750 2150
Wire Wire Line
	4750 2150 5000 2150
Connection ~ 4900 1900
Text HLabel 3600 3250 0    60   Input ~ 0
CPU_TX
Text HLabel 3600 2500 0    60   Output ~ 0
CPU_RX
Text HLabel 3650 4850 0    60   Input ~ 0
~CPU_TX_EN
Text HLabel 1900 1500 0    60   UnSpc ~ 0
GND
Text HLabel 1900 1100 0    60   UnSpc ~ 0
+5V
Wire Wire Line
	3900 1900 4900 1900
Connection ~ 3900 1100
$Comp
L SN75176BDR U407
U 1 1 59177EF3
P 8250 2800
F 0 "U407" H 8300 3250 60  0000 L CNN
F 1 "SN75176BDR" H 8300 3150 60  0000 L CNN
F 2 "smd-normal:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 8250 2800 60  0001 C CNN
F 3 "" H 8250 2800 60  0000 C CNN
	1    8250 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 3050 7550 2600
Wire Wire Line
	7550 2600 7950 2600
Wire Wire Line
	7350 2600 7350 3000
Wire Wire Line
	7350 3000 7950 3000
Wire Wire Line
	7950 2900 7800 2900
Wire Wire Line
	7800 2700 7800 4850
Wire Wire Line
	7950 2700 7800 2700
Connection ~ 7800 2900
Wire Wire Line
	6150 1100 6150 4650
$Comp
L C C409
U 1 1 5917845D
P 6350 3450
F 0 "C409" V 6098 3450 50  0000 C CNN
F 1 "100n" V 6189 3450 50  0000 C CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 6388 3300 30  0001 C CNN
F 3 "" H 6350 3450 60  0000 C CNN
	1    6350 3450
	0    1    1    0   
$EndComp
$Comp
L GNDA #PWR305
U 1 1 591784F4
P 6500 3600
F 0 "#PWR305" H 6500 3350 50  0001 C CNN
F 1 "GNDA" H 6505 3427 50  0000 C CNN
F 2 "" H 6500 3600 60  0000 C CNN
F 3 "" H 6500 3600 60  0000 C CNN
	1    6500 3600
	1    0    0    -1  
$EndComp
Connection ~ 6150 2900
$Comp
L C C411
U 1 1 5917860E
P 8450 1800
F 0 "C411" H 8335 1754 50  0000 R CNN
F 1 "100n" H 8335 1845 50  0000 R CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 8488 1650 30  0001 C CNN
F 3 "" H 8450 1800 60  0000 C CNN
	1    8450 1800
	-1   0    0    1   
$EndComp
Wire Wire Line
	8250 1100 8250 2500
Wire Wire Line
	8450 1650 8450 1400
Wire Wire Line
	8450 1400 8250 1400
Connection ~ 8250 1400
$Comp
L GNDA #PWR309
U 1 1 59178885
P 8450 1950
F 0 "#PWR309" H 8450 1700 50  0001 C CNN
F 1 "GNDA" H 8455 1777 50  0000 C CNN
F 2 "" H 8450 1950 60  0000 C CNN
F 3 "" H 8450 1950 60  0000 C CNN
	1    8450 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 1500 6600 1500
$Comp
L GNDA #PWR308
U 1 1 591789EC
P 8250 3100
F 0 "#PWR308" H 8250 2850 50  0001 C CNN
F 1 "GNDA" H 8255 2927 50  0000 C CNN
F 2 "" H 8250 3100 60  0000 C CNN
F 3 "" H 8250 3100 60  0000 C CNN
	1    8250 3100
	1    0    0    -1  
$EndComp
$Comp
L Wago_233-502 P402
U 1 1 59178A7E
P 9200 2800
F 0 "P402" H 9278 2841 50  0000 L CNN
F 1 "Wago_233-502" H 9278 2750 50  0000 L CNN
F 2 "th-npth:Wire-2_Wago-233-P2.5mm" H 9200 2800 60  0001 C CNN
F 3 "" H 9200 2800 60  0000 C CNN
	1    9200 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 2700 8800 2700
Wire Wire Line
	8800 2700 8800 2750
Wire Wire Line
	8800 2750 9000 2750
Wire Wire Line
	9000 2850 8800 2850
Wire Wire Line
	8800 2850 8800 2900
Wire Wire Line
	8800 2900 8650 2900
$Comp
L Cosmo_1010-817 U406
U 1 1 59178DD0
P 5300 4750
F 0 "U406" H 5300 5075 50  0000 C CNN
F 1 "Cosmo_1010-817" H 5300 4984 50  0000 C CNN
F 2 "th-normal:DIP-4_W300mil" H 5100 4550 50  0001 L CIN
F 3 "" H 5300 4650 50  0000 L CNN
	1    5300 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 4850 5600 4850
$Comp
L C C410
U 1 1 59178F22
P 6350 4350
F 0 "C410" V 6098 4350 50  0000 C CNN
F 1 "100n" V 6189 4350 50  0000 C CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 6388 4200 30  0001 C CNN
F 3 "" H 6350 4350 60  0000 C CNN
	1    6350 4350
	0    1    1    0   
$EndComp
$Comp
L GNDA #PWR306
U 1 1 59179160
P 6500 4450
F 0 "#PWR306" H 6500 4200 50  0001 C CNN
F 1 "GNDA" H 6505 4277 50  0000 C CNN
F 2 "" H 6500 4450 60  0000 C CNN
F 3 "" H 6500 4450 60  0000 C CNN
	1    6500 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 3450 6500 3600
Wire Wire Line
	6500 4350 6500 4450
Wire Wire Line
	6150 3450 6200 3450
Wire Wire Line
	6150 4350 6200 4350
Connection ~ 6150 3450
Wire Wire Line
	6150 4650 5600 4650
Connection ~ 6150 4350
$Comp
L GNDA #PWR307
U 1 1 591794D3
P 7350 3600
F 0 "#PWR307" H 7350 3350 50  0001 C CNN
F 1 "GNDA" H 7355 3427 50  0000 C CNN
F 2 "" H 7350 3600 60  0000 C CNN
F 3 "" H 7350 3600 60  0000 C CNN
	1    7350 3600
	1    0    0    -1  
$EndComp
$Comp
L R R407
U 1 1 5917952C
P 7550 3500
F 0 "R407" V 7450 3500 50  0000 C CNN
F 1 "10k" V 7550 3500 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 7480 3500 30  0001 C CNN
F 3 "" H 7550 3500 30  0000 C CNN
	1    7550 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	7350 3600 7350 3500
Wire Wire Line
	7350 3500 7400 3500
Wire Wire Line
	7700 3500 7800 3500
Connection ~ 7800 3500
$Comp
L R R403
U 1 1 5917984B
P 4750 4850
F 0 "R403" V 4650 4850 50  0000 C CNN
F 1 "R" V 4750 4850 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 4680 4850 30  0001 C CNN
F 3 "" H 4750 4850 30  0000 C CNN
	1    4750 4850
	0    1    1    0   
$EndComp
Connection ~ 3900 2000
Wire Wire Line
	3900 4650 5000 4650
Connection ~ 3900 3050
Wire Wire Line
	3650 4850 4600 4850
Wire Wire Line
	4900 4850 5000 4850
Text Notes 3950 2850 0    60   ~ 0
LED on when log. 0\nLED off when log. 1
Text Notes 9100 2600 0    60   ~ 0
RS485
$Comp
L Wago_233-502 P403
U 1 1 5917A251
P 10100 1150
F 0 "P403" H 10178 1191 50  0000 L CNN
F 1 "Wago_233-502" H 10178 1100 50  0000 L CNN
F 2 "th-npth:Wire-2_Wago-233-P2.5mm" H 10100 1150 60  0001 C CNN
F 3 "" H 10100 1150 60  0000 C CNN
	1    10100 1150
	1    0    0    -1  
$EndComp
Connection ~ 8250 1100
$Comp
L GNDA #PWR310
U 1 1 5917A355
P 9000 1400
F 0 "#PWR310" H 9000 1150 50  0001 C CNN
F 1 "GNDA" H 9005 1227 50  0000 C CNN
F 2 "" H 9000 1400 60  0000 C CNN
F 3 "" H 9000 1400 60  0000 C CNN
	1    9000 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 1200 9000 1400
Wire Wire Line
	9900 1200 9000 1200
$Comp
L FUSE F402
U 1 1 5917A6EB
P 9150 1100
F 0 "F402" V 8943 1100 50  0000 C CNN
F 1 "FUSE" V 9034 1100 50  0000 C CNN
F 2 "th-normal:Fuse_5x20_Schurter-0031-8201" V 9080 1100 30  0001 C CNN
F 3 "" H 9150 1100 30  0000 C CNN
	1    9150 1100
	0    1    1    0   
$EndComp
Wire Wire Line
	9300 1100 9900 1100
Wire Wire Line
	5700 1100 9000 1100
Wire Wire Line
	1900 1100 3050 1100
$Comp
L R R401
U 1 1 591869D7
P 3000 1400
F 0 "R401" V 2900 1400 50  0000 C CNN
F 1 "10k" V 3000 1400 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 2930 1400 30  0001 C CNN
F 3 "" H 3000 1400 30  0000 C CNN
	1    3000 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 1100 2750 1400
Wire Wire Line
	2750 1400 2850 1400
Connection ~ 2750 1100
Wire Wire Line
	3150 1400 3250 1400
Wire Wire Line
	3250 1400 3250 1900
Wire Wire Line
	3250 1900 2750 1900
$Comp
L IRLML6402 Q402
U 1 1 59186E57
P 3250 1200
F 0 "Q402" V 3578 1200 50  0000 C CNN
F 1 "IRLML6402" V 3487 1200 50  0000 C CNN
F 2 "smd-handsolder:SOT-23_HandSoldering" H 3450 1300 29  0001 C CNN
F 3 "" H 3250 1200 60  0000 C CNN
	1    3250 1200
	0    1    -1   0   
$EndComp
Text HLabel 2750 1900 0    60   Input ~ 0
~READER_ON
$Comp
L R R410
U 1 1 5918D8C0
P 5900 2350
F 0 "R410" V 5800 2350 50  0000 C CNN
F 1 "1k" V 5900 2350 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 5830 2350 30  0001 C CNN
F 3 "" H 5900 2350 30  0000 C CNN
	1    5900 2350
	0    1    1    0   
$EndComp
$Comp
L R R411
U 1 1 5918D906
P 6750 2100
F 0 "R411" H 6820 2146 50  0000 L CNN
F 1 "1k" H 6820 2055 50  0000 L CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 6680 2100 30  0001 C CNN
F 3 "" H 6750 2100 30  0000 C CNN
	1    6750 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 1800 6750 1800
Wire Wire Line
	6750 1800 6750 1950
Connection ~ 6350 1800
Wire Wire Line
	6750 2250 6750 2350
Wire Wire Line
	6750 2350 6350 2350
Connection ~ 6350 2350
Wire Wire Line
	6050 2350 6100 2350
Wire Wire Line
	6100 2350 6100 2600
Connection ~ 6100 2600
Connection ~ 5650 2350
$Comp
L R R408
U 1 1 5918DF82
P 4150 2200
F 0 "R408" V 4250 2100 50  0000 L CNN
F 1 "1k" V 4150 2150 50  0000 L CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 4080 2200 30  0001 C CNN
F 3 "" H 4150 2200 30  0000 C CNN
	1    4150 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	3900 2200 4000 2200
Connection ~ 3900 2200
Wire Wire Line
	4300 2200 4450 2200
Connection ~ 4450 2200
$Comp
L R R409
U 1 1 5918E10D
P 4700 3500
F 0 "R409" V 4600 3500 50  0000 C CNN
F 1 "1k" V 4700 3500 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 4630 3500 30  0001 C CNN
F 3 "" H 4700 3500 30  0000 C CNN
	1    4700 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 3250 4950 3500
Wire Wire Line
	4950 3500 4850 3500
Connection ~ 4950 3250
Wire Wire Line
	4450 3250 4450 3500
Wire Wire Line
	4450 3500 4550 3500
Connection ~ 4450 3250
Text Notes 9800 950  0    60   ~ 0
Power for reader
$Comp
L Jamicon_220uF_25V C406
U 1 1 59172B3A
P 2300 1300
F 0 "C406" H 2350 1400 50  0000 L CNN
F 1 "Jamicon_220uF_25V" H 2100 1000 50  0000 L CNN
F 2 "th-normal:C_Radial_D8_L11.5_P3.5" H 2300 1300 60  0001 C CNN
F 3 "" H 2300 1300 60  0000 C CNN
	1    2300 1300
	1    0    0    -1  
$EndComp
$Comp
L Jamicon_220uF_25V C301
U 1 1 59180D99
P 6600 1300
F 0 "C301" H 6650 1400 50  0000 L CNN
F 1 "Jamicon_220uF_25V" H 6700 1150 50  0000 L CNN
F 2 "th-normal:C_Radial_D8_L11.5_P3.5" H 6600 1300 60  0001 C CNN
F 3 "" H 6600 1300 60  0000 C CNN
	1    6600 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 1150 6600 1100
Connection ~ 6600 1100
Wire Wire Line
	6600 1500 6600 1450
Text Label 3750 1100 0    60   ~ 0
DCDCIN5V
Text Label 8700 2700 0    60   ~ 0
485A
Text Label 8700 2900 0    60   ~ 0
485B
$EndSCHEMATC
