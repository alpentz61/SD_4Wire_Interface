EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLedger 17000 11000
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
L 4wire_sd_breakout:EK-TM4C1294XL_RevD_Breadboard_Header J2
U 1 1 60EE9458
P 10900 5550
F 0 "J2" H 10900 8075 50  0000 C CNN
F 1 "EK-TM4C1294XL_RevD_Breadboard_Header" H 10900 7984 50  0000 C CNN
F 2 "4wire_sd_breakout:EK-TM4C1294XL_RevD_Breadboard_Header" H 10300 5950 50  0001 C CNN
F 3 "~" H 10300 5950 50  0001 C CNN
	1    10900 5550
	-1   0    0    1   
$EndComp
Text Label 10100 7650 2    50   ~ 0
CMD_CLK
Text Label 10100 7550 2    50   ~ 0
CMD_CS
Text Label 10100 7450 2    50   ~ 0
CMD_MOSI
Text Label 10100 7350 2    50   ~ 0
CMD_MISO
$Comp
L 4wire_sd_breakout:CMOD_Generic U4
U 1 1 60EF1B6F
P 7100 5700
F 0 "U4" H 7100 7025 50  0000 C CNN
F 1 "CMOD_Generic" H 7100 6934 50  0000 C CNN
F 2 "4wire_sd_breakout:CMOD_Generic" H 6700 5700 50  0001 C CNN
F 3 "~" H 6700 5700 50  0001 C CNN
	1    7100 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 6800 8100 6800
Wire Wire Line
	7750 6700 8100 6700
Wire Wire Line
	7750 6600 8100 6600
Wire Wire Line
	7750 6900 7750 7050
$Comp
L power:GND #PWR0101
U 1 1 60F1335D
P 7750 7050
F 0 "#PWR0101" H 7750 6800 50  0001 C CNN
F 1 "GND" H 7755 6877 50  0000 C CNN
F 2 "" H 7750 7050 50  0001 C CNN
F 3 "" H 7750 7050 50  0001 C CNN
	1    7750 7050
	1    0    0    -1  
$EndComp
Text Label 7750 5800 0    50   ~ 0
SD_CLK
Text Label 7750 5400 0    50   ~ 0
CMD_CLK
Text Label 7750 6200 0    50   ~ 0
SD_DAT0
Text Label 7750 6300 0    50   ~ 0
SD_DAT1
Text Label 7750 5900 0    50   ~ 0
SD_DAT2
Text Label 7750 6000 0    50   ~ 0
SD_DAT3
Text Label 7750 6100 0    50   ~ 0
SD_CMD
Text Label 7750 5000 0    50   ~ 0
DATA_FSS
Text Label 7750 5200 0    50   ~ 0
DATA_D1
Text Label 7750 5300 0    50   ~ 0
DATA_D0
Text Label 7750 5600 0    50   ~ 0
DATA_D2
Wire Wire Line
	9150 5600 9150 5850
Wire Wire Line
	9150 5850 10100 5850
Wire Wire Line
	7750 5600 9150 5600
Wire Wire Line
	9350 5500 9350 5750
Wire Wire Line
	9350 5750 10100 5750
Wire Wire Line
	7750 5500 9350 5500
Wire Wire Line
	9650 5100 9650 5650
Wire Wire Line
	9650 5650 10100 5650
Wire Wire Line
	7750 5100 9650 5100
Wire Wire Line
	9750 5000 9750 5350
Wire Wire Line
	9750 5350 10100 5350
Wire Wire Line
	7750 5000 9750 5000
Wire Wire Line
	9550 5200 9550 5450
Wire Wire Line
	9550 5450 10100 5450
Wire Wire Line
	7750 5200 9550 5200
Wire Wire Line
	9450 5300 9450 5550
Wire Wire Line
	9450 5550 10100 5550
Wire Wire Line
	7750 5300 9450 5300
Wire Wire Line
	7750 5400 9250 5400
Text Label 7750 6500 0    50   ~ 0
CMD_CS
Wire Wire Line
	10100 7750 9750 7750
Wire Wire Line
	9750 7750 9750 7800
$Comp
L power:GND #PWR0102
U 1 1 60F30A2F
P 9750 7800
F 0 "#PWR0102" H 9750 7550 50  0001 C CNN
F 1 "GND" H 9755 7627 50  0000 C CNN
F 2 "" H 9750 7800 50  0001 C CNN
F 3 "" H 9750 7800 50  0001 C CNN
	1    9750 7800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 60F327DC
P 12100 7750
F 0 "#PWR0103" H 12100 7500 50  0001 C CNN
F 1 "GND" H 12105 7577 50  0000 C CNN
F 2 "" H 12100 7750 50  0001 C CNN
F 3 "" H 12100 7750 50  0001 C CNN
	1    12100 7750
	1    0    0    -1  
$EndComp
Wire Wire Line
	12100 7750 11700 7750
Text Label 7750 6400 0    50   ~ 0
CMD_MOSI
Text Label 7750 5700 0    50   ~ 0
CMD_MISO
Wire Wire Line
	7750 5700 9050 5700
Text Label 7750 6800 0    50   ~ 0
Sig_0
Text Label 7750 6700 0    50   ~ 0
Sig_1
Text Label 7750 6600 0    50   ~ 0
Sig_2
Wire Wire Line
	10100 7250 9750 7250
Wire Wire Line
	10100 7150 9750 7150
Wire Wire Line
	10100 7050 9750 7050
Wire Wire Line
	10100 6950 9750 6950
Wire Wire Line
	10100 6850 9750 6850
Wire Wire Line
	10100 6750 9750 6750
Text Label 9800 7250 0    50   ~ 0
Sig_0
Text Label 9800 7150 0    50   ~ 0
Sig_1
Text Label 9800 7050 0    50   ~ 0
Sig_2
Text Label 9800 6950 0    50   ~ 0
Sig_3
Text Label 9800 6850 0    50   ~ 0
Sig_4
Text Label 9800 6750 0    50   ~ 0
Sig_5
Text Label 8150 1200 1    50   ~ 0
CMD_CLK
Text Label 8350 1200 1    50   ~ 0
CMD_CS
Text Label 7750 1200 1    50   ~ 0
CMD_MISO
Text Label 7950 1200 1    50   ~ 0
CMD_MOSI
Text Label 8350 2100 3    50   ~ 0
DATA_FSS
Text Label 7750 5100 0    50   ~ 0
DATA_CLK
Text Label 8150 2100 3    50   ~ 0
DATA_CLK
Text Label 7950 2100 3    50   ~ 0
DATA_D0
Text Label 7750 2100 3    50   ~ 0
DATA_D1
Text Label 7550 2100 3    50   ~ 0
DATA_D2
Text Label 7750 5500 0    50   ~ 0
DATA_D3
Text Label 7350 2100 3    50   ~ 0
DATA_D3
NoConn ~ 7750 4600
NoConn ~ 7750 4700
NoConn ~ 7750 4800
NoConn ~ 7750 4900
NoConn ~ 6450 4600
NoConn ~ 6450 4700
NoConn ~ 6450 4800
NoConn ~ 6450 4900
Text Label 7550 1200 1    50   ~ 0
Sig_0
Text Label 7350 1200 1    50   ~ 0
Sig_1
Text Label 7150 1200 1    50   ~ 0
Sig_2
Text Notes 5700 1500 0    50   ~ 0
TM4 to FPGA \nTest Signal Header\n
Text Label 6950 1200 1    50   ~ 0
Sig_3
Text Label 7150 2300 1    50   ~ 0
Sig_4
Text Label 6950 2300 1    50   ~ 0
Sig_5
Text Notes 850  10200 0    50   ~ 0
Notes on the Signal Routing:\n1.  Pins 26 through 46 are all in IO bank 34 of the FPGA, so no IO Bank crossings are needed for these critical signals.\n2.  Pin 1, 2, 3, 4, 45, 45, 47, and 48 are not used, \n    so that we can interchangeably duse both the 48 pin CMOD A7 as well as the 40 pin CMOD C2 \n    (for the CMOD C2, these 8 pins will not be used).\n3.  Within the remaining bank 34 pins (26 throuh 44), the only Multi Region Clock        \n    Capable Pins are 36, 40, and 43.  These pins have been assigned to the SD bus clock,\n    command bus clock, and data bus clock respectively, so that these pins have\n    appropriate timing.  Also, this will allow the entire FPGA to be clocked from the data\n    or command bus signals, removing the need for an internal clock (after cell configuration).\n4.  After accounting for all signals in data quad SPI, cmd normal SPI, and SD bus signals, the last 3 bank 34 \n    pins have been assigned to general purpose signals (signal 0, 1, 2). \n5.  Additional signals, as well as debug header signals, have been assigned to pins which are not located in bank 34, \n    with the idea that their timing is less critical to the overall operation. \n
Wire Wire Line
	9250 7650 10100 7650
Wire Wire Line
	9250 5400 9250 7650
Wire Wire Line
	6450 5000 6100 5000
Wire Wire Line
	6450 5100 6100 5100
Wire Wire Line
	6450 5200 6100 5200
Text Label 6150 5000 0    50   ~ 0
Sig_3
Text Label 6150 5100 0    50   ~ 0
Sig_4
Text Label 6150 5200 0    50   ~ 0
Sig_5
$Comp
L 4wire_sd_breakout:LA2016_Logic_analyzer_header U3
U 1 1 6128A72A
P 4700 7450
F 0 "U3" V 3535 7450 50  0000 C CNN
F 1 "LA2016_Logic_analyzer_header" V 3626 7450 50  0000 C CNN
F 2 "4wire_sd_breakout:LA2016_Logic_Analyzer_Connector" H 4700 6900 50  0001 C CNN
F 3 "" H 6075 7350 50  0001 C CNN
	1    4700 7450
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 6128E896
P 5100 6750
F 0 "#PWR0106" H 5100 6500 50  0001 C CNN
F 1 "GND" V 5105 6622 50  0000 R CNN
F 2 "" H 5100 6750 50  0001 C CNN
F 3 "" H 5100 6750 50  0001 C CNN
	1    5100 6750
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 6128EF52
P 4300 6750
F 0 "#PWR0107" H 4300 6500 50  0001 C CNN
F 1 "GND" V 4305 6622 50  0000 R CNN
F 2 "" H 4300 6750 50  0001 C CNN
F 3 "" H 4300 6750 50  0001 C CNN
	1    4300 6750
	0    1    1    0   
$EndComp
Wire Wire Line
	5100 7350 5500 7350
Wire Wire Line
	5100 7550 5500 7550
Wire Wire Line
	5100 7750 5500 7750
Wire Wire Line
	5100 7950 5500 7950
Wire Wire Line
	5100 8150 5500 8150
Wire Wire Line
	5100 8350 5500 8350
Text Label 5150 8350 0    50   ~ 0
SD_CLK
Text Label 5150 8150 0    50   ~ 0
SD_CMD
Text Label 5150 7950 0    50   ~ 0
SD_DAT0
Text Label 5150 7750 0    50   ~ 0
SD_DAT1
Text Label 5150 7550 0    50   ~ 0
SD_DAT2
Text Label 5150 7350 0    50   ~ 0
SD_DAT3
NoConn ~ 5100 6550
NoConn ~ 4300 6550
NoConn ~ 4300 6950
NoConn ~ 4300 7150
NoConn ~ 4300 7350
NoConn ~ 4300 7550
NoConn ~ 4300 7750
NoConn ~ 4300 7950
Text Notes 4100 6200 0    50   ~ 0
FPGA to SD Card\nTest Header
Wire Wire Line
	8850 7550 8850 6500
Wire Wire Line
	8850 7550 10100 7550
Wire Wire Line
	8950 7450 10100 7450
Wire Wire Line
	8950 7450 8950 6400
Wire Wire Line
	9050 7350 9050 5700
Wire Wire Line
	9050 7350 10100 7350
Wire Wire Line
	8250 5900 8250 7200
Wire Wire Line
	7750 5900 8250 5900
Wire Wire Line
	8350 6000 8350 7275
Wire Wire Line
	7750 6000 8350 6000
Wire Wire Line
	8450 6100 8450 7350
Wire Wire Line
	7750 6100 8450 6100
Wire Wire Line
	7750 5800 8550 5800
Wire Wire Line
	7750 6200 8650 6200
Wire Wire Line
	7750 6300 8750 6300
Wire Wire Line
	8450 7350 8150 7350
Wire Wire Line
	8350 7275 8050 7275
Wire Wire Line
	8250 7200 7925 7200
Wire Wire Line
	8550 5800 8550 7425
Wire Wire Line
	7750 6500 8850 6500
Wire Wire Line
	7750 6400 8950 6400
Wire Wire Line
	8150 7350 8150 7600
Wire Wire Line
	8050 7275 8050 7475
Wire Wire Line
	7925 7200 7925 7350
Text Label 8750 8250 1    50   ~ 0
SD_DAT1
Text Label 8650 8250 1    50   ~ 0
SD_DAT0
Text Label 7925 8250 1    50   ~ 0
SD_DAT2
$Comp
L Device:R R10
U 1 1 61354848
P 8750 8450
F 0 "R10" H 8700 8600 50  0000 L CNN
F 1 "200" H 8650 8300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8680 8450 50  0001 C CNN
F 3 "~" H 8750 8450 50  0001 C CNN
	1    8750 8450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 61354086
P 8650 8450
F 0 "R9" H 8600 8600 50  0000 L CNN
F 1 "200" H 8550 8300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8580 8450 50  0001 C CNN
F 3 "~" H 8650 8450 50  0001 C CNN
	1    8650 8450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 61353D86
P 8150 8450
F 0 "R8" H 8100 8600 50  0000 L CNN
F 1 "200" H 8050 8300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8080 8450 50  0001 C CNN
F 3 "~" H 8150 8450 50  0001 C CNN
	1    8150 8450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 613533F5
P 8050 8450
F 0 "R7" H 8000 8600 50  0000 L CNN
F 1 "200" H 7950 8300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7980 8450 50  0001 C CNN
F 3 "~" H 8050 8450 50  0001 C CNN
	1    8050 8450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 6134EE91
P 7925 8450
F 0 "R6" H 7875 8600 50  0000 L CNN
F 1 "200" H 7825 8300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7855 8450 50  0001 C CNN
F 3 "~" H 7925 8450 50  0001 C CNN
	1    7925 8450
	1    0    0    -1  
$EndComp
Text Label 8150 8250 1    50   ~ 0
SD_CMD
Text Label 8050 8250 1    50   ~ 0
SD_DAT3
$Comp
L power:+3.3V #PWR0108
U 1 1 61319818
P 8400 8500
F 0 "#PWR0108" H 8400 8350 50  0001 C CNN
F 1 "+3.3V" H 8415 8673 50  0000 C CNN
F 2 "" H 8400 8500 50  0001 C CNN
F 3 "" H 8400 8500 50  0001 C CNN
	1    8400 8500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 613CB9BD
P 7425 7350
F 0 "R1" V 7400 7475 50  0000 L CNN
F 1 "50k" V 7400 7075 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7355 7350 50  0001 C CNN
F 3 "~" H 7425 7350 50  0001 C CNN
	1    7425 7350
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 613CD703
P 7425 7475
F 0 "R2" V 7400 7600 50  0000 L CNN
F 1 "50k" V 7400 7200 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7355 7475 50  0001 C CNN
F 3 "~" H 7425 7475 50  0001 C CNN
	1    7425 7475
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 613CDA97
P 7425 7600
F 0 "R3" V 7400 7725 50  0000 L CNN
F 1 "50k" V 7400 7325 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7355 7600 50  0001 C CNN
F 3 "~" H 7425 7600 50  0001 C CNN
	1    7425 7600
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 613CDD28
P 7425 7725
F 0 "R4" V 7400 7850 50  0000 L CNN
F 1 "50k" V 7400 7450 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7355 7725 50  0001 C CNN
F 3 "~" H 7425 7725 50  0001 C CNN
	1    7425 7725
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 613CE012
P 7425 7850
F 0 "R5" V 7400 7975 50  0000 L CNN
F 1 "50k" V 7400 7575 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7355 7850 50  0001 C CNN
F 3 "~" H 7425 7850 50  0001 C CNN
	1    7425 7850
	0    1    1    0   
$EndComp
Wire Wire Line
	8750 6300 8750 7850
Wire Wire Line
	8650 6200 8650 7725
Wire Wire Line
	7575 7350 7925 7350
Connection ~ 7925 7350
Wire Wire Line
	7925 7350 7925 8300
Wire Wire Line
	7575 7475 8050 7475
Connection ~ 8050 7475
Wire Wire Line
	8050 7475 8050 8300
Wire Wire Line
	7575 7600 8150 7600
Connection ~ 8150 7600
Wire Wire Line
	8150 7600 8150 8300
Wire Wire Line
	7575 7725 8650 7725
Connection ~ 8650 7725
Wire Wire Line
	8650 7725 8650 8300
Wire Wire Line
	7575 7850 8750 7850
Connection ~ 8750 7850
Wire Wire Line
	8750 7850 8750 8300
Wire Wire Line
	7275 7350 7000 7350
Wire Wire Line
	7000 7350 7000 7475
Wire Wire Line
	7000 7475 7275 7475
Wire Wire Line
	7275 7600 7000 7600
Wire Wire Line
	7000 7600 7000 7475
Connection ~ 7000 7475
Wire Wire Line
	7275 7725 7000 7725
Connection ~ 7000 7600
Wire Wire Line
	7275 7850 7000 7850
Wire Wire Line
	7000 7600 7000 7725
Connection ~ 7000 7725
Wire Wire Line
	7000 7725 7000 7850
$Comp
L power:+3.3V #PWR0109
U 1 1 61408A02
P 7000 7350
F 0 "#PWR0109" H 7000 7200 50  0001 C CNN
F 1 "+3.3V" H 7015 7523 50  0000 C CNN
F 2 "" H 7000 7350 50  0001 C CNN
F 3 "" H 7000 7350 50  0001 C CNN
	1    7000 7350
	1    0    0    -1  
$EndComp
Connection ~ 7000 7350
$Comp
L power:+5V #PWR0110
U 1 1 61409F24
P 10100 7850
F 0 "#PWR0110" H 10100 7700 50  0001 C CNN
F 1 "+5V" V 10115 7978 50  0000 L CNN
F 2 "" H 10100 7850 50  0001 C CNN
F 3 "" H 10100 7850 50  0001 C CNN
	1    10100 7850
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0111
U 1 1 6140CAAB
P 5975 6900
F 0 "#PWR0111" H 5975 6750 50  0001 C CNN
F 1 "+5V" H 5990 7073 50  0000 C CNN
F 2 "" H 5975 6900 50  0001 C CNN
F 3 "" H 5975 6900 50  0001 C CNN
	1    5975 6900
	1    0    0    -1  
$EndComp
$Comp
L Device:D D1
U 1 1 6140EC5B
P 6125 6900
F 0 "D1" H 6125 6800 50  0000 C CNN
F 1 "D" H 6125 6850 50  0000 C CNN
F 2 "Diode_THT:D_A-405_P12.70mm_Horizontal" H 6125 6900 50  0001 C CNN
F 3 "~" H 6125 6900 50  0001 C CNN
	1    6125 6900
	-1   0    0    1   
$EndComp
Wire Wire Line
	7925 8600 7925 8975
Wire Wire Line
	8750 8900 8750 8600
Wire Wire Line
	8650 8900 8650 8600
$Comp
L power:GND #PWR0112
U 1 1 61316908
P 8550 8900
F 0 "#PWR0112" H 8550 8650 50  0001 C CNN
F 1 "GND" H 8550 8775 50  0000 C CNN
F 2 "" H 8550 8900 50  0001 C CNN
F 3 "" H 8550 8900 50  0001 C CNN
	1    8550 8900
	-1   0    0    1   
$EndComp
Wire Wire Line
	8150 8600 8150 8900
Wire Wire Line
	8050 8600 8050 8900
$Comp
L 4wire_sd_breakout:SD_Card_Bare_Attachment_Pads U6
U 1 1 613042DE
P 7850 10050
F 0 "U6" H 8828 10679 50  0000 L CNN
F 1 "SD_Card_Bare_Attachment_Pads" H 8828 10588 50  0000 L CNN
F 2 "4wire_sd_breakout:SD_Card_Bare_Wire_Attachment_Pads" H 8500 9875 50  0001 C CNN
F 3 "" H 7850 10300 50  0001 C CNN
	1    7850 10050
	1    0    0    -1  
$EndComp
Text Label 8450 8250 1    50   ~ 0
SD_CLK
Wire Wire Line
	8550 7425 8450 7425
Wire Wire Line
	8450 7425 8450 8900
Wire Wire Line
	8200 8900 8250 8900
$Comp
L power:GND #PWR0113
U 1 1 614414A6
P 8300 8800
F 0 "#PWR0113" H 8300 8550 50  0001 C CNN
F 1 "GND" H 8375 8775 50  0000 R CNN
F 2 "" H 8300 8800 50  0001 C CNN
F 3 "" H 8300 8800 50  0001 C CNN
	1    8300 8800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 8900 8400 8900
Wire Wire Line
	8300 8800 8200 8800
Wire Wire Line
	8200 8800 8200 8900
$Comp
L Regulator_Linear:LM7805_TO220 U1
U 1 1 6145B3A6
P 2650 2075
F 0 "U1" H 2650 2317 50  0000 C CNN
F 1 "LM7805_TO220" H 2650 2226 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 2650 2300 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 2650 2025 50  0001 C CNN
	1    2650 2075
	1    0    0    -1  
$EndComp
Wire Wire Line
	6275 6900 6450 6900
NoConn ~ 6450 5700
NoConn ~ 6450 5800
NoConn ~ 6450 5900
NoConn ~ 6450 6000
NoConn ~ 6450 6100
NoConn ~ 6450 6200
NoConn ~ 6450 6300
NoConn ~ 6450 6400
NoConn ~ 6450 6500
NoConn ~ 6450 6600
NoConn ~ 6450 6700
NoConn ~ 6450 6800
$Comp
L power:GND #PWR0114
U 1 1 614B9939
P 10100 3150
F 0 "#PWR0114" H 10100 2900 50  0001 C CNN
F 1 "GND" V 10105 3022 50  0000 R CNN
F 2 "" H 10100 3150 50  0001 C CNN
F 3 "" H 10100 3150 50  0001 C CNN
	1    10100 3150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 614BAA7F
P 10100 4850
F 0 "#PWR0115" H 10100 4600 50  0001 C CNN
F 1 "GND" V 10105 4722 50  0000 R CNN
F 2 "" H 10100 4850 50  0001 C CNN
F 3 "" H 10100 4850 50  0001 C CNN
	1    10100 4850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 614BC54E
P 10100 6150
F 0 "#PWR0116" H 10100 5900 50  0001 C CNN
F 1 "GND" V 10105 6022 50  0000 R CNN
F 2 "" H 10100 6150 50  0001 C CNN
F 3 "" H 10100 6150 50  0001 C CNN
	1    10100 6150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 614BD1F5
P 11700 4850
F 0 "#PWR0117" H 11700 4600 50  0001 C CNN
F 1 "GND" V 11705 4722 50  0000 R CNN
F 2 "" H 11700 4850 50  0001 C CNN
F 3 "" H 11700 4850 50  0001 C CNN
	1    11700 4850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 614BDF08
P 11700 6150
F 0 "#PWR0118" H 11700 5900 50  0001 C CNN
F 1 "GND" V 11705 6022 50  0000 R CNN
F 2 "" H 11700 6150 50  0001 C CNN
F 3 "" H 11700 6150 50  0001 C CNN
	1    11700 6150
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C5
U 1 1 612D4CC5
P 8300 8675
F 0 "C5" V 8125 8675 50  0000 C CNN
F 1 "0.1 uF" V 8200 8700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 8300 8675 50  0001 C CNN
F 3 "~" H 8300 8675 50  0001 C CNN
	1    8300 8675
	0    1    1    0   
$EndComp
Wire Wire Line
	8400 8500 8400 8675
Wire Wire Line
	8400 8675 8400 8900
Connection ~ 8400 8675
Wire Wire Line
	8200 8675 8200 8800
Connection ~ 8200 8800
$Comp
L Device:C_Small C4
U 1 1 612E5E69
P 4575 2225
F 0 "C4" H 4625 2300 50  0000 C CNN
F 1 "10uF" H 4575 2175 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 4575 2225 50  0001 C CNN
F 3 "~" H 4575 2225 50  0001 C CNN
	1    4575 2225
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C3
U 1 1 612E995D
P 3825 2225
F 0 "C3" H 3875 2300 50  0000 C CNN
F 1 "0.1uF" H 3825 2175 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 3825 2225 50  0001 C CNN
F 3 "~" H 3825 2225 50  0001 C CNN
	1    3825 2225
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 61352FDC
P 4200 2425
F 0 "#PWR0119" H 4200 2175 50  0001 C CNN
F 1 "GND" H 4205 2252 50  0000 C CNN
F 2 "" H 4200 2425 50  0001 C CNN
F 3 "" H 4200 2425 50  0001 C CNN
	1    4200 2425
	1    0    0    -1  
$EndComp
$Comp
L 4wire_sd_breakout:LD1117V33 U2
U 1 1 612D0AB5
P 4200 2150
F 0 "U2" H 4200 2467 50  0000 C CNN
F 1 "LD1117V33" H 4200 2376 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4200 1825 50  0001 C CNN
F 3 "" H 4200 2150 50  0001 C CNN
	1    4200 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4575 2125 4575 2075
Wire Wire Line
	4575 2075 4500 2075
Wire Wire Line
	3900 2075 3825 2075
Wire Wire Line
	3825 2075 3825 2125
Wire Wire Line
	4200 2375 4200 2425
Wire Wire Line
	4200 2375 4575 2375
Wire Wire Line
	4575 2375 4575 2325
Connection ~ 4200 2375
Wire Wire Line
	4200 2375 3825 2375
Wire Wire Line
	3825 2375 3825 2325
$Comp
L power:GND #PWR0120
U 1 1 6137A48D
P 2650 2400
F 0 "#PWR0120" H 2650 2150 50  0001 C CNN
F 1 "GND" H 2655 2227 50  0000 C CNN
F 2 "" H 2650 2400 50  0001 C CNN
F 3 "" H 2650 2400 50  0001 C CNN
	1    2650 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 6137B0AC
P 3025 2225
F 0 "C2" H 3075 2300 50  0000 C CNN
F 1 "0.1uF" H 3025 2175 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 3025 2225 50  0001 C CNN
F 3 "~" H 3025 2225 50  0001 C CNN
	1    3025 2225
	-1   0    0    1   
$EndComp
Wire Wire Line
	3025 2125 3025 2075
Wire Wire Line
	3025 2075 2950 2075
$Comp
L Device:C_Small C1
U 1 1 61382AC5
P 2275 2225
F 0 "C1" H 2325 2300 50  0000 C CNN
F 1 "0.33uF" H 2275 2175 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 2275 2225 50  0001 C CNN
F 3 "~" H 2275 2225 50  0001 C CNN
	1    2275 2225
	-1   0    0    1   
$EndComp
Wire Wire Line
	2275 2125 2275 2075
Wire Wire Line
	2275 2075 2350 2075
Connection ~ 4575 2075
$Comp
L power:+3.3V #PWR0121
U 1 1 6139222C
P 4575 2075
F 0 "#PWR0121" H 4575 1925 50  0001 C CNN
F 1 "+3.3V" H 4590 2248 50  0000 C CNN
F 2 "" H 4575 2075 50  0001 C CNN
F 3 "" H 4575 2075 50  0001 C CNN
	1    4575 2075
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0122
U 1 1 6139523C
P 3025 2075
F 0 "#PWR0122" H 3025 1925 50  0001 C CNN
F 1 "+5V" H 3040 2248 50  0000 C CNN
F 2 "" H 3025 2075 50  0001 C CNN
F 3 "" H 3025 2075 50  0001 C CNN
	1    3025 2075
	1    0    0    -1  
$EndComp
Connection ~ 3025 2075
Wire Wire Line
	2275 1550 2275 2075
Connection ~ 2275 2075
Wire Wire Line
	3825 1550 3825 2075
Connection ~ 3825 2075
$Comp
L power:GND #PWR0123
U 1 1 613DCF8D
P 3325 1325
F 0 "#PWR0123" H 3325 1075 50  0001 C CNN
F 1 "GND" H 3330 1152 50  0000 C CNN
F 2 "" H 3325 1325 50  0001 C CNN
F 3 "" H 3325 1325 50  0001 C CNN
	1    3325 1325
	1    0    0    -1  
$EndComp
Wire Wire Line
	3325 1325 3325 1275
Wire Wire Line
	3325 1275 3325 1125
Connection ~ 3325 1275
Wire Wire Line
	3100 1550 3825 1550
Wire Wire Line
	3100 1550 2275 1550
Connection ~ 3100 1550
$Comp
L 4wire_sd_breakout:DC_Barrel_Jack J1
U 1 1 613B26B1
P 3100 1200
F 0 "J1" V 3540 1392 50  0000 C CNN
F 1 "DC_Barrel_Jack" V 3449 1392 50  0000 C CNN
F 2 "4wire_sd_breakout:DC_Barrel_Adapter" H 3075 1500 50  0001 C CNN
F 3 "" H 3125 1250 50  0001 C CNN
	1    3100 1200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2275 2325 2275 2375
Wire Wire Line
	2275 2375 2650 2375
Wire Wire Line
	3025 2325 3025 2375
Wire Wire Line
	3025 2375 2650 2375
Connection ~ 2650 2375
Wire Wire Line
	2650 2400 2650 2375
$Comp
L 4wire_sd_breakout:USB3300_Breakout_Header U7
U 1 1 61383659
P 12000 1650
F 0 "U7" H 10800 1925 50  0000 L CNN
F 1 "USB3300_Breakout_Header" H 11500 1650 50  0000 L CNN
F 2 "4wire_sd_breakout:LA2016_Logic_Analyzer_Connector" H 12000 2150 50  0001 C CNN
F 3 "" H 12000 1650 50  0001 C CNN
	1    12000 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	12900 5750 11700 5750
Wire Wire Line
	12800 5650 12800 2200
Wire Wire Line
	12800 2200 12700 2200
Wire Wire Line
	11700 5650 12800 5650
Wire Wire Line
	12700 2050 12700 2200
Wire Wire Line
	12900 2050 12900 5750
Wire Wire Line
	12500 2050 12500 2300
Wire Wire Line
	12500 2300 12700 2300
Wire Wire Line
	12700 2300 12700 5550
Wire Wire Line
	11700 5550 12700 5550
Wire Wire Line
	12600 5450 12600 2400
Wire Wire Line
	12600 2400 12300 2400
Wire Wire Line
	12300 2400 12300 2050
Wire Wire Line
	11700 5450 12600 5450
Wire Wire Line
	12500 4650 12500 2500
Wire Wire Line
	12500 2500 12100 2500
Wire Wire Line
	12100 2500 12100 2050
Wire Wire Line
	11700 4650 12500 4650
Wire Wire Line
	11700 3950 12400 3950
Wire Wire Line
	12400 3950 12400 2600
Wire Wire Line
	12400 2600 11900 2600
Wire Wire Line
	11900 2600 11900 2050
Wire Wire Line
	10100 3950 9550 3950
Wire Wire Line
	9550 3950 9550 2600
Wire Wire Line
	9550 2600 11700 2600
Wire Wire Line
	11700 2600 11700 2050
Wire Wire Line
	10100 4050 9450 4050
Wire Wire Line
	9450 4050 9450 2500
Wire Wire Line
	9450 2500 11500 2500
Wire Wire Line
	11500 2500 11500 2050
$Comp
L power:GND #PWR0124
U 1 1 61430DC1
P 11300 2050
F 0 "#PWR0124" H 11300 1800 50  0001 C CNN
F 1 "GND" H 11305 1877 50  0000 C CNN
F 2 "" H 11300 2050 50  0001 C CNN
F 3 "" H 11300 2050 50  0001 C CNN
	1    11300 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 614325CD
P 11300 1250
F 0 "#PWR0125" H 11300 1000 50  0001 C CNN
F 1 "GND" H 11305 1077 50  0000 C CNN
F 2 "" H 11300 1250 50  0001 C CNN
F 3 "" H 11300 1250 50  0001 C CNN
	1    11300 1250
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0126
U 1 1 614346D5
P 12900 1250
F 0 "#PWR0126" H 12900 1100 50  0001 C CNN
F 1 "+5V" H 12915 1423 50  0000 C CNN
F 2 "" H 12900 1250 50  0001 C CNN
F 3 "" H 12900 1250 50  0001 C CNN
	1    12900 1250
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0127
U 1 1 61435D3F
P 12700 1250
F 0 "#PWR0127" H 12700 1100 50  0001 C CNN
F 1 "+5V" H 12715 1423 50  0000 C CNN
F 2 "" H 12700 1250 50  0001 C CNN
F 3 "" H 12700 1250 50  0001 C CNN
	1    12700 1250
	1    0    0    -1  
$EndComp
NoConn ~ 12500 1250
$Comp
L power:+3.3V #PWR0128
U 1 1 6144B81C
P 11100 1250
F 0 "#PWR0128" H 11100 1100 50  0001 C CNN
F 1 "+3.3V" H 11115 1423 50  0000 C CNN
F 2 "" H 11100 1250 50  0001 C CNN
F 3 "" H 11100 1250 50  0001 C CNN
	1    11100 1250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0129
U 1 1 6144C94E
P 11100 2050
F 0 "#PWR0129" H 11100 1900 50  0001 C CNN
F 1 "+3.3V" H 11115 2223 50  0000 C CNN
F 2 "" H 11100 2050 50  0001 C CNN
F 3 "" H 11100 2050 50  0001 C CNN
	1    11100 2050
	-1   0    0    1   
$EndComp
Wire Wire Line
	12100 1200 12100 1250
Wire Wire Line
	12100 1200 12000 1200
Wire Wire Line
	12000 1200 12000 2700
Wire Wire Line
	12000 2700 12300 2700
Wire Wire Line
	12300 2700 12300 4450
Wire Wire Line
	11700 4450 12300 4450
Wire Wire Line
	12300 900  12300 1250
Wire Wire Line
	11900 900  11900 1250
Wire Wire Line
	11700 900  11700 1250
Wire Wire Line
	11500 900  11500 1250
Text Label 11750 4650 0    50   ~ 0
USB0_DAT4
Wire Wire Line
	11700 4550 12150 4550
Wire Wire Line
	11700 4350 12150 4350
Wire Wire Line
	11700 4250 12150 4250
Text Label 11750 5450 0    50   ~ 0
USB0_DAT3
Text Label 11750 5550 0    50   ~ 0
USB0_DAT2
Text Label 11750 5650 0    50   ~ 0
USB0_DAT1
Text Label 11750 5750 0    50   ~ 0
USB0_DAT0
Text Label 11750 4550 0    50   ~ 0
USB0_STP
Text Label 11750 4450 0    50   ~ 0
USB0_CLK
Text Label 11750 4350 0    50   ~ 0
USB0_NXT
Text Label 11750 4250 0    50   ~ 0
USB0_DIR
Text Label 11500 1200 1    50   ~ 0
USB0_STP
Text Label 11700 1200 1    50   ~ 0
USB0_NXT
Text Label 11900 1200 1    50   ~ 0
USB0_DIR
Wire Wire Line
	12150 5850 11700 5850
Text Label 11750 5850 0    50   ~ 0
USB0_RST
Text Label 12300 1200 1    50   ~ 0
USB0_RST
NoConn ~ 11700 4750
NoConn ~ 11700 4950
NoConn ~ 11700 5050
NoConn ~ 11700 5150
NoConn ~ 11700 5250
NoConn ~ 11700 5350
NoConn ~ 11700 7850
NoConn ~ 11700 7650
NoConn ~ 11700 7550
NoConn ~ 11700 7450
NoConn ~ 11700 7350
NoConn ~ 11700 7250
NoConn ~ 11700 7150
NoConn ~ 11700 7050
NoConn ~ 11700 6950
NoConn ~ 11700 6850
NoConn ~ 11700 6750
NoConn ~ 11700 6650
NoConn ~ 11700 6550
NoConn ~ 11700 6450
NoConn ~ 11700 6350
NoConn ~ 11700 6250
NoConn ~ 11700 6050
NoConn ~ 11700 5950
NoConn ~ 11700 4150
NoConn ~ 11700 4050
NoConn ~ 11700 3850
NoConn ~ 11700 3750
NoConn ~ 11700 3050
NoConn ~ 11700 3150
NoConn ~ 11700 3250
NoConn ~ 11700 3350
NoConn ~ 11700 3450
NoConn ~ 11700 3550
NoConn ~ 11700 3650
Wire Notes Line
	12400 7900 16450 7900
Wire Notes Line
	12400 3000 16450 3000
Wire Notes Line
	12400 3000 12400 7900
Text Notes 13500 7850 0    50   ~ 0
Component Keepout (TM4C1294 body above this location)\n
Text Notes 13500 3100 0    50   ~ 0
Component Keepout (TM4C1294 body above this location)\n
$Comp
L 4wire_sd_breakout:LA2016_Logic_analyzer_header U8
U 1 1 616E37FB
P 14800 1650
F 0 "U8" H 13600 1900 50  0000 L CNN
F 1 "LA2016_Logic_analyzer_header" H 14250 1650 50  0000 L CNN
F 2 "4wire_sd_breakout:LA2016_Logic_Analyzer_Connector" H 14800 1100 50  0001 C CNN
F 3 "" H 16175 1550 50  0001 C CNN
	1    14800 1650
	1    0    0    -1  
$EndComp
Text Label 15700 2100 3    50   ~ 0
USB0_DAT0
Wire Wire Line
	15100 900  15100 1250
Wire Wire Line
	14900 900  14900 1250
Wire Wire Line
	14700 900  14700 1250
Wire Wire Line
	14500 900  14500 1250
Wire Wire Line
	14300 1250 14300 900 
NoConn ~ 13900 1250
NoConn ~ 13900 2050
Wire Wire Line
	15700 2050 15700 2400
Wire Wire Line
	15500 2050 15500 2400
Wire Wire Line
	15300 2050 15300 2400
Wire Wire Line
	15100 2050 15100 2400
Wire Wire Line
	14900 2050 14900 2400
Wire Wire Line
	14700 2050 14700 2400
Wire Wire Line
	14500 2050 14500 2400
Wire Wire Line
	14300 2050 14300 2400
Text Label 15500 2100 3    50   ~ 0
USB0_DAT1
Text Label 15300 2100 3    50   ~ 0
USB0_DAT2
Text Label 15100 2100 3    50   ~ 0
USB0_DAT3
Text Label 14900 2100 3    50   ~ 0
USB0_DAT4
Text Label 11750 3950 0    50   ~ 0
USB0_DAT5
Text Label 14700 2100 3    50   ~ 0
USB0_DAT5
Text Label 9650 3950 0    50   ~ 0
USB0_DAT6
Text Label 9650 4050 0    50   ~ 0
USB0_DAT7
Text Label 14300 2100 3    50   ~ 0
USB0_DAT7
Text Label 14500 2100 3    50   ~ 0
USB0_DAT6
$Comp
L power:GND #PWR0130
U 1 1 618C2BCA
P 14100 1250
F 0 "#PWR0130" H 14100 1000 50  0001 C CNN
F 1 "GND" H 14105 1077 50  0000 C CNN
F 2 "" H 14100 1250 50  0001 C CNN
F 3 "" H 14100 1250 50  0001 C CNN
	1    14100 1250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 618C44F4
P 14100 2050
F 0 "#PWR0131" H 14100 1800 50  0001 C CNN
F 1 "GND" H 14105 1877 50  0000 C CNN
F 2 "" H 14100 2050 50  0001 C CNN
F 3 "" H 14100 2050 50  0001 C CNN
	1    14100 2050
	1    0    0    -1  
$EndComp
Text Label 14300 1200 1    50   ~ 0
USB0_CLK
Text Label 14900 1200 1    50   ~ 0
USB0_DIR
Text Label 14700 1200 1    50   ~ 0
USB0_NXT
Text Label 14500 1200 1    50   ~ 0
USB0_STP
Text Label 15100 1200 1    50   ~ 0
USB0_RST
NoConn ~ 15700 1250
NoConn ~ 15500 1250
NoConn ~ 15300 1250
NoConn ~ 7350 4050
NoConn ~ 10100 3050
NoConn ~ 10100 3250
NoConn ~ 10100 3350
NoConn ~ 10100 3450
NoConn ~ 10100 3550
NoConn ~ 10100 3650
NoConn ~ 10100 3750
NoConn ~ 10100 3850
NoConn ~ 10100 4150
NoConn ~ 10100 4250
NoConn ~ 10100 4350
NoConn ~ 10100 4450
NoConn ~ 10100 4550
NoConn ~ 10100 4650
NoConn ~ 10100 4750
NoConn ~ 10100 4950
NoConn ~ 10100 5050
NoConn ~ 10100 5150
NoConn ~ 10100 5250
NoConn ~ 10100 5950
NoConn ~ 10100 6050
NoConn ~ 10100 6250
NoConn ~ 7750 1300
Wire Wire Line
	6950 900  6950 1250
Wire Wire Line
	7150 900  7150 1250
Wire Wire Line
	7350 900  7350 1250
Wire Wire Line
	7550 900  7550 1250
Wire Wire Line
	7750 900  7750 1250
Wire Wire Line
	7950 900  7950 1250
NoConn ~ 6550 2050
Wire Wire Line
	8150 900  8150 1250
$Comp
L power:GND #PWR0104
U 1 1 6123202F
P 6750 2050
F 0 "#PWR0104" H 6750 1800 50  0001 C CNN
F 1 "GND" H 6755 1877 50  0000 C CNN
F 2 "" H 6750 2050 50  0001 C CNN
F 3 "" H 6750 2050 50  0001 C CNN
	1    6750 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 2050 6950 2400
Wire Wire Line
	7150 2050 7150 2400
Wire Wire Line
	7350 2050 7350 2400
Wire Wire Line
	7550 2050 7550 2400
Wire Wire Line
	7750 2050 7750 2400
Wire Wire Line
	7950 2050 7950 2400
Wire Wire Line
	8150 2050 8150 2400
Wire Wire Line
	8350 2050 8350 2400
NoConn ~ 6550 1250
Wire Wire Line
	8350 900  8350 1250
$Comp
L 4wire_sd_breakout:LA2016_Logic_analyzer_header U5
U 1 1 6122A93E
P 7450 1650
F 0 "U5" H 6250 2000 50  0000 L CNN
F 1 "LA2016_Logic_analyzer_header" H 6850 1700 50  0000 L CNN
F 2 "4wire_sd_breakout:LA2016_Logic_Analyzer_Connector" H 7450 1100 50  0001 C CNN
F 3 "" H 8825 1550 50  0001 C CNN
	1    7450 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 61232AEF
P 6750 1250
F 0 "#PWR0105" H 6750 1000 50  0001 C CNN
F 1 "GND" H 6755 1077 50  0000 C CNN
F 2 "" H 6750 1250 50  0001 C CNN
F 3 "" H 6750 1250 50  0001 C CNN
	1    6750 1250
	-1   0    0    1   
$EndComp
NoConn ~ 4300 8150
NoConn ~ 4300 8350
NoConn ~ 5100 6950
NoConn ~ 5100 7150
NoConn ~ 6450 5600
NoConn ~ 6450 5500
NoConn ~ 6450 5400
NoConn ~ 6450 5300
NoConn ~ 10100 6350
NoConn ~ 10100 6450
NoConn ~ 10100 6550
NoConn ~ 10100 6650
$EndSCHEMATC