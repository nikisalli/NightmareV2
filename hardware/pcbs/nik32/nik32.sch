EESchema Schematic File Version 4
LIBS:nik32-cache
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
L z_esp32:ESP-32S M1
U 1 1 5CDF0DAD
P 6000 2900
F 0 "M1" H 6000 4137 60  0000 C CNN
F 1 "ESP-32S" H 6000 4031 60  0000 C CNN
F 2 "nik_things:esp32_board" H 6050 4050 60  0001 C CNN
F 3 "" H 5650 3150 60  0001 C CNN
	1    6000 2900
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM32F4:STM32F411RETx U1
U 1 1 5CDF0C44
P 2350 3950
F 0 "U1" H 2350 2064 50  0000 C CNN
F 1 "STM32F411RETx" H 2350 1973 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 1750 2250 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00115249.pdf" H 2350 3950 50  0001 C CNN
	1    2350 3950
	1    0    0    -1  
$EndComp
$EndSCHEMATC
