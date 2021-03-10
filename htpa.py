from pymata4 import pymata4
from sensordef import *
import time
import numpy as np
import struct
import os

def read_eeprom(ee_regADD):
    board.i2c_write(EEPROM_ADDRESS, [ee_regADD>>8, ee_regADD])
    board.i2c_read(EEPROM_ADDRESS, register=None, number_of_bytes=1, callback=eeprom_callback)
    time.sleep(0.015)

def eeprom_callback(data):
    global eeprom
    eeprom.append(data[-2])

def wake_sensor():
    board.i2c_write(SENSOR_ADDRESS, [CONFIGURATION_REGISTER, 0x01])

def write_sensor_calib_settings():
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER1, mbit_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER2, bias_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER3, bias_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER4, clk_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER5, bpa_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER6, bpa_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER7, pu_calib])

def start_sensor():
    board.i2c_write(SENSOR_ADDRESS, [CONFIGURATION_REGISTER, 0x09])

def to_float(num):
    num = str(bin(num))[2:]
    return float(struct.unpack('!f',struct.pack('!I', int(num, 2)))[0])

# def calculate_pixcij():
#     pixcij_int32 = np.zeros(shape=(32,32))
#     for m in range(32):
#         for n in range(32):
#             pixcij_int32[m][n] = pixcmax - pixcmin
#             pixcij_int32[m][n] = pixcij_int32[m][n] / 65535
#             pixcij_int32[m][n] = pixcij_int32[m][n] * 

def main(board):
    start_time = time.time()

    #initialize i2c pins
    print("Initializing...")
    board.set_pin_mode_i2c()
    time.sleep(5)

    #set sensor characteristics
    number_row = 32
    number_col = 32
    number_blocks = 8
    number_pixel = 1024

    #read eeprom data
    global eeprom

    if os.path.isfile("./eeprom.txt"):
        with open('eeprom.txt', 'r') as filehandle:
            filecontents = filehandle.readlines()
            for line in filecontents:
                value = int(line[:-1]) # remove last character which is newline
                eeprom.append(value)
    else:
        print("Reading EEPROM data...")
        for i in range(0x1F3F):
            read_eeprom(i)

        with open('eeprom.txt', 'w') as filehandle:
            for value in eeprom:
                filehandle.write('%s\n' %value)

    print(eeprom)
    print(len(eeprom))

    id = eeprom[E_ID4]<<24 | eeprom[E_ID3]<<16 | eeprom[E_ID2]<<8 | eeprom[E_ID1]

    mbit_calib = eeprom[E_MBIT_CALIB]
    bias_calib = eeprom[E_BIAS_CALIB]
    clk_calib = eeprom[E_CLK_CALIB]
    bpa_calib = eeprom[E_BPA_CALIB]
    pu_calib = eeprom[E_PU_CALIB]

    mbit_user = eeprom[E_MBIT_USER]
    bias_user = eeprom[E_BIAS_USER]
    clk_user = eeprom[E_CLK_USER]
    bpa_user = eeprom[E_BPA_USER]
    pu_user = eeprom[E_PU_USER]

    vddth1 = eeprom[E_VDDTH1_2]<<8 | eeprom[E_VDDTH1_1]
    vddth2 = eeprom[E_VDDTH2_2]<<8 | eeprom[E_VDDTH2_1]
    vddscgrad = eeprom[E_VDDSCGRAD]
    vddscoff = eeprom[E_VDDSCOFF]

    ptatth1 = eeprom[E_PTATTH1_2]<<8 | eeprom[E_PTATTH1_1]
    ptatth2 = eeprom[E_PTATTH2_2]<<8 | eeprom[E_PTATTH2_1]

    nrofdefpix = eeprom[E_NROFDEFPIX]
    gradscale = eeprom[E_GRADSCALE]
    tablenumber = eeprom[E_TABLENUMBER2]<<8 | eeprom[E_TABLENUMBER1]
    arraytype = eeprom[E_ARRAYTYPE]

    ptatgr_float = to_float(eeprom[E_PTATGR_4]<<24 | eeprom[E_PTATGR_3]<<16 | eeprom[E_PTATGR_2]<<8 | eeprom[E_PTATGR_1])
    ptatoff_float = to_float(eeprom[E_PTATOFF_4]<<24 | eeprom[E_PTATOFF_3]<<16 | eeprom[E_PTATOFF_2]<<8 | eeprom[E_PTATOFF_1])

    pixcmax = to_float(eeprom[E_PIXCMAX_4]<<24 | eeprom[E_PIXCMAX_3]<<16 | eeprom[E_PIXCMAX_2]<<8 | eeprom[E_PIXCMAX_1])
    pixcmin = to_float(eeprom[E_PIXCMIN_4]<<24 | eeprom[E_PIXCMIN_3]<<16 | eeprom[E_PIXCMIN_2]<<8 | eeprom[E_PIXCMIN_1])

    epsilon = eeprom[E_EPSILON]
    globaloff = eeprom[E_GLOBALOFF]
    globalgain = eeprom[E_GLOBALGAIN_2]<<8 | eeprom[E_GLOBALGAIN_1]

    #-----DeadPixAdr-----
    deadpixadr = np.zeros(nrofdefpix)
    for i in range(nrofdefpix):
        deadpixadr[i] = eeprom[E_DEADPIXADR + 2 * i + 1]<<8 | eeprom[E_DEADPIXADR + 2 * i]
        if deadpixadr[i]>512:
            deadpixadr[i] = 1024 + 512 - deadpixadr[i] + 2 * (deadpixadr[i] % 32) - 32

    #-----DeadPixMask-----
    deadpixmask = np.zeros(nrofdefpix)
    for i in range(nrofdefpix):
        deadpixmask[i] = eeprom[E_DEADPIXMASK + i]

    #-----Thgrad_ij-----
    m = 0
    n = 0
    addr_i = 0x0740 #start address
    thgrad = np.zeros(shape=(32,32))

    #top half
    for i in range(512):
        addr_i = 0x0740 + 2 * i
        thgrad[m][n] = eeprom[E_THGRAD + 2 * i + 1]<<8 | eeprom[E_THGRAD + 2 * i]
        n += 1
        if n==32:
            n = 0
            m += 1

    #bottom half
    for i in range(number_col):
        thgrad[31][i] = eeprom[E_THGRAD + 0x0400 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0400 + 2 * i]
        thgrad[30][i] = eeprom[E_THGRAD + 0x0400 + 1 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0400 + 1 * 64 + 2 * i]
        thgrad[29][i] = eeprom[E_THGRAD + 0x0400 + 2 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0400 + 2 * 64 + 2 * i]
        thgrad[28][i] = eeprom[E_THGRAD + 0x0400 + 3 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0400 + 3 * 64 + 2 * i]

        thgrad[27][i] = eeprom[E_THGRAD + 0x0500 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0500 + 2 * i]
        thgrad[26][i] = eeprom[E_THGRAD + 0x0500 + 1 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0500 + 1 * 64 + 2 * i]
        thgrad[25][i] = eeprom[E_THGRAD + 0x0500 + 2 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0500 + 2 * 64 + 2 * i]
        thgrad[24][i] = eeprom[E_THGRAD + 0x0500 + 3 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0500 + 3 * 64 + 2 * i]

        thgrad[23][i] = eeprom[E_THGRAD + 0x0600 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0600 + 2 * i]
        thgrad[22][i] = eeprom[E_THGRAD + 0x0600 + 1 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0600 + 1 * 64 + 2 * i]
        thgrad[21][i] = eeprom[E_THGRAD + 0x0600 + 2 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0600 + 2 * 64 + 2 * i]
        thgrad[20][i] = eeprom[E_THGRAD + 0x0600 + 3 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0600 + 3 * 64 + 2 * i]

        thgrad[19][i] = eeprom[E_THGRAD + 0x0700 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0700 + 2 * i]
        thgrad[18][i] = eeprom[E_THGRAD + 0x0700 + 1 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0700 + 1 * 64 + 2 * i]
        thgrad[17][i] = eeprom[E_THGRAD + 0x0700 + 2 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0700 + 2 * 64 + 2 * i]
        thgrad[16][i] = eeprom[E_THGRAD + 0x0700 + 3 * 64 + 2 * i]<<8 | eeprom[E_THGRAD + 0x0700 + 3 * 64 + 2 * i]

    #-----Thoffset_ij-----
    

    #wake sensor
    #wake_sensor()

    #write calibration settings to sensor
    #write_sensor_calib_settings()

    #start sensor
    #start_sensor()

    #other calculations before main loop
    gradscale_div = 2**eeprom[E_GRADSCALE]
    vddscgrad_div = 2**eeprom[E_VDDSCGRAD]
    vddscoff_div = 2**eeprom[E_VDDSCOFF]

    
    
    print(ptatoff_float)
    print(ptatgr_float)
    print(pixcmax)
    print(pixcmin)
    print(id)

    print(f"Execution duration: {time.time()-start_time}")



    
board = pymata4.Pymata4()
try:
    eeprom = []
    main(board)
except KeyboardInterrupt:
    board.shutdown()
finally:
    board.shutdown()