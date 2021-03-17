from pymata4 import pymata4
from sensordef import *
import time
import numpy as np
import struct
import os

def read_eeprom(ee_regADD):
    board.i2c_write(EEPROM_ADDRESS, [ee_regADD>>8, ee_regADD])
    time.sleep(0.015)
    board.i2c_read(EEPROM_ADDRESS, register=None, number_of_bytes=1, callback=eeprom_callback)
    time.sleep(0.015)

def eeprom_callback(data):
    global eeprom
    eeprom.append(data[-2])

def wake_sensor():
    board.i2c_write(SENSOR_ADDRESS, [CONFIGURATION_REGISTER, 0x01])

def start_sensor():
    board.i2c_write(SENSOR_ADDRESS, [CONFIGURATION_REGISTER, 0x09])

def to_float(num):
    num = str(bin(num))[2:]
    return float(struct.unpack('!f',struct.pack('!I', int(num, 2)))[0])

def calc_timer_duration(bw, clk, mbit):
    Fclk_float = 12000000 / 63 * clk + 1000000 #calc clk in Hz
    a = 1/ NORM_BW
    b = 32 * (2 ** (mbit & 0b00001111) + 4) / Fclk_float
    c = b / a
    c /= bw
    c *= SAFETY_FAC
    calculated_timer_duration = c * 1000000 #c in  s | timer_duration in µs
    return calculated_timer_duration

def read_sensor(regADD, num_bytes, cb):
    board.i2c_read_restart_transmission(SENSOR_ADDRESS, register=regADD, number_of_bytes=num_bytes, callback=cb)
    time.sleep(2)

def cb_statusreg(data):
    global statusreg
    print(data)
    statusreg = data[-2]

def cb_data_top_block0(data):
    global data_top_block0
    print(data)

def main(board):
    start_time = time.time()

    #initialize global variables
    global eeprom

    # global data_top_block0, data_top_block1, data_top_block2, data_top_block3
    # global data_bottom_block0, data_bottom_block1, data_bottom_block2, data_bottom_block3
    # global electrical_offset_top, electrical_offset_bottom
    # global eloffset
    # global ptat_top_block0, ptat_top_block1, ptat_top_block2, ptat_top_block3
    # global ptat_bottom_block0, ptat_bottom_block1, ptat_bottom_block2, ptat_bottom_block3
    # global vdd_top_block0, vdd_top_block1, vdd_top_block2, vdd_top_block3
    # global vdd_bottom_block0, vdd_bottom_block1, vdd_bottom_block2, vdd_bottom_block3
    # global data_pixel
    # global statusreg

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
    if os.path.isfile("./eeprom.txt"):
        print("Getting EEPROM from cache...")
        with open('eeprom.txt', 'r') as filehandle:
            filecontents = filehandle.readlines()
            for line in filecontents:
                value = int(line[:-1]) # remove last character which is newline
                eeprom.append(value)
    else:
        print("Reading EEPROM data...")
        for i in range(0x1F3F+1):
            read_eeprom(i)

        print("Caching...")
        with open('eeprom.txt', 'w') as filehandle:
            for value in eeprom:
                filehandle.write('%s\n' %value)

    #print(eeprom)
    print(f"EEPROM length: {len(eeprom)}")

    bw = (eeprom[E_BW2]<<8 | eeprom[E_BW1]) / 100
    device_id = eeprom[E_ID4]<<24 | eeprom[E_ID3]<<16 | eeprom[E_ID2]<<8 | eeprom[E_ID1] #changed from id

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
        thgrad[30][i] = eeprom[E_THGRAD + 0x0400 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0400 + 1 * 64 + 2 * i]
        thgrad[29][i] = eeprom[E_THGRAD + 0x0400 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0400 + 2 * 64 + 2 * i]
        thgrad[28][i] = eeprom[E_THGRAD + 0x0400 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0400 + 3 * 64 + 2 * i]

        thgrad[27][i] = eeprom[E_THGRAD + 0x0500 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0500 + 2 * i]
        thgrad[26][i] = eeprom[E_THGRAD + 0x0500 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0500 + 1 * 64 + 2 * i]
        thgrad[25][i] = eeprom[E_THGRAD + 0x0500 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0500 + 2 * 64 + 2 * i]
        thgrad[24][i] = eeprom[E_THGRAD + 0x0500 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0500 + 3 * 64 + 2 * i]

        thgrad[23][i] = eeprom[E_THGRAD + 0x0600 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0600 + 2 * i]
        thgrad[22][i] = eeprom[E_THGRAD + 0x0600 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0600 + 1 * 64 + 2 * i]
        thgrad[21][i] = eeprom[E_THGRAD + 0x0600 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0600 + 2 * 64 + 2 * i]
        thgrad[20][i] = eeprom[E_THGRAD + 0x0600 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0600 + 3 * 64 + 2 * i]

        thgrad[19][i] = eeprom[E_THGRAD + 0x0700 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0700 + 2 * i]
        thgrad[18][i] = eeprom[E_THGRAD + 0x0700 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0700 + 1 * 64 + 2 * i]
        thgrad[17][i] = eeprom[E_THGRAD + 0x0700 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0700 + 2 * 64 + 2 * i]
        thgrad[16][i] = eeprom[E_THGRAD + 0x0700 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_THGRAD + 0x0700 + 3 * 64 + 2 * i]

    #-----Thoffset_ij-----
    m = 0
    n = 0
    thoffset = np.zeros(shape=(32,32))
    #top half
    for i in range(512):
        addr_i = 0x0F40 + 2 * i
        thoffset[m][n] = eeprom[E_THOFFSET + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 2 * i]
        n += 1
        if n==32:
            n = 0
            m += 1
    #bottom half
    for i in range(number_col):
        thoffset[31][i] = eeprom[E_THOFFSET + 0x0400 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0400 + 2 * i]
        thoffset[30][i] = eeprom[E_THOFFSET + 0x0400 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0400 + 1 * 64 + 2 * i]
        thoffset[29][i] = eeprom[E_THOFFSET + 0x0400 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0400 + 2 * 64 + 2 * i]
        thoffset[28][i] = eeprom[E_THOFFSET + 0x0400 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0400 + 3 * 64 + 2 * i]

        thoffset[27][i] = eeprom[E_THOFFSET + 0x0500 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0500 + 2 * i]
        thoffset[26][i] = eeprom[E_THOFFSET + 0x0500 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0500 + 1 * 64 + 2 * i]
        thoffset[25][i] = eeprom[E_THOFFSET + 0x0500 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0500 + 2 * 64 + 2 * i]
        thoffset[24][i] = eeprom[E_THOFFSET + 0x0500 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0500 + 3 * 64 + 2 * i]

        thoffset[23][i] = eeprom[E_THOFFSET + 0x0600 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0600 + 2 * i]
        thoffset[22][i] = eeprom[E_THOFFSET + 0x0600 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0600 + 1 * 64 + 2 * i]
        thoffset[21][i] = eeprom[E_THOFFSET + 0x0600 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0600 + 2 * 64 + 2 * i]
        thoffset[20][i] = eeprom[E_THOFFSET + 0x0600 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0600 + 3 * 64 + 2 * i]

        thoffset[19][i] = eeprom[E_THOFFSET + 0x0700 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0700 + 2 * i]
        thoffset[18][i] = eeprom[E_THOFFSET + 0x0700 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0700 + 1 * 64 + 2 * i]
        thoffset[17][i] = eeprom[E_THOFFSET + 0x0700 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0700 + 2 * 64 + 2 * i]
        thoffset[16][i] = eeprom[E_THOFFSET + 0x0700 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_THOFFSET + 0x0700 + 3 * 64 + 2 * i]

    #-----VddCompGrad-----
    vddcompgrad = np.zeros(shape=(8,32))
    for i in range(number_col):
        #top half
        vddcompgrad[0][i] = eeprom[E_VDDCOMPGRAD + 2 * i + 1]<<8 | eeprom[E_VDDCOMPGRAD + 2 * i]
        vddcompgrad[1][i] = eeprom[E_VDDCOMPGRAD + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPGRAD + 1 * 64 + 2 * i]
        vddcompgrad[2][i] = eeprom[E_VDDCOMPGRAD + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPGRAD + 2 * 64 + 2 * i]
        vddcompgrad[3][i] = eeprom[E_VDDCOMPGRAD + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPGRAD + 3 * 64 + 2 * i]
        #bottom half (backwards)
        vddcompgrad[7][i] = eeprom[E_VDDCOMPGRAD + 4 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPGRAD + 4 * 64 + 2 * i]
        vddcompgrad[6][i] = eeprom[E_VDDCOMPGRAD + 5 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPGRAD + 5 * 64 + 2 * i]
        vddcompgrad[5][i] = eeprom[E_VDDCOMPGRAD + 6 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPGRAD + 6 * 64 + 2 * i]
        vddcompgrad[4][i] = eeprom[E_VDDCOMPGRAD + 7 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPGRAD + 7 * 64 + 2 * i]

    #-----VddCompOff-----
    vddcompoff = np.zeros(shape=(8,32))
    for i in range(number_col):
        #top half
        vddcompoff[0][i] = eeprom[E_VDDCOMPOFF + 2 * i + 1]<<8 | eeprom[E_VDDCOMPOFF + 2 * i]
        vddcompoff[1][i] = eeprom[E_VDDCOMPOFF + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPOFF + 1 * 64 + 2 * i]
        vddcompoff[2][i] = eeprom[E_VDDCOMPOFF + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPOFF + 2 * 64 + 2 * i]
        vddcompoff[3][i] = eeprom[E_VDDCOMPOFF + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPOFF + 3 * 64 + 2 * i]
        #bottom half (backwards)
        vddcompoff[7][i] = eeprom[E_VDDCOMPOFF + 4 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPOFF + 4 * 64 + 2 * i]
        vddcompoff[6][i] = eeprom[E_VDDCOMPOFF + 5 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPOFF + 5 * 64 + 2 * i]
        vddcompoff[5][i] = eeprom[E_VDDCOMPOFF + 6 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPOFF + 6 * 64 + 2 * i]
        vddcompoff[4][i] = eeprom[E_VDDCOMPOFF + 7 * 64 + 2 * i + 1]<<8 | eeprom[E_VDDCOMPOFF + 7 * 64 + 2 * i]

    #-----P_ij-----
    m = 0
    n = 0
    pij = np.zeros(shape=(32,32))
    #top half
    for i in range(512):
        addr_i = 0x0F40 + 2 * i
        pij[m][n] = eeprom[E_PIJ + 2 * i + 1]<<8 | eeprom[E_PIJ + 2 * i]
        n += 1
        if n==32:
            n = 0
            m += 1
    #bottom half
    for i in range(number_col):
        pij[31][i] = eeprom[E_PIJ + 0x0400 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0400 + 2 * i]
        pij[30][i] = eeprom[E_PIJ + 0x0400 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0400 + 1 * 64 + 2 * i]
        pij[29][i] = eeprom[E_PIJ + 0x0400 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0400 + 2 * 64 + 2 * i]
        pij[28][i] = eeprom[E_PIJ + 0x0400 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0400 + 3 * 64 + 2 * i]

        pij[27][i] = eeprom[E_PIJ + 0x0500 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0500 + 2 * i]
        pij[26][i] = eeprom[E_PIJ + 0x0500 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0500 + 1 * 64 + 2 * i]
        pij[25][i] = eeprom[E_PIJ + 0x0500 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0500 + 2 * 64 + 2 * i]
        pij[24][i] = eeprom[E_PIJ + 0x0500 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0500 + 3 * 64 + 2 * i]

        pij[23][i] = eeprom[E_PIJ + 0x0600 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0600 + 2 * i]
        pij[22][i] = eeprom[E_PIJ + 0x0600 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0600 + 1 * 64 + 2 * i]
        pij[21][i] = eeprom[E_PIJ + 0x0600 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0600 + 2 * 64 + 2 * i]
        pij[20][i] = eeprom[E_PIJ + 0x0600 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0600 + 3 * 64 + 2 * i]

        pij[19][i] = eeprom[E_PIJ + 0x0700 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0700 + 2 * i]
        pij[18][i] = eeprom[E_PIJ + 0x0700 + 1 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0700 + 1 * 64 + 2 * i]
        pij[17][i] = eeprom[E_PIJ + 0x0700 + 2 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0700 + 2 * 64 + 2 * i]
        pij[16][i] = eeprom[E_PIJ + 0x0700 + 3 * 64 + 2 * i + 1]<<8 | eeprom[E_PIJ + 0x0700 + 3 * 64 + 2 * i]

    #wake sensor
    board.i2c_write(SENSOR_ADDRESS, [CONFIGURATION_REGISTER, 0x01])

    #write calibration settings to sensor
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER1, mbit_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER2, bias_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER3, bias_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER4, clk_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER5, bpa_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER6, bpa_calib])
    board.i2c_write(SENSOR_ADDRESS, [TRIM_REGISTER7, pu_calib])

    #start sensor
    board.i2c_write(SENSOR_ADDRESS, [CONFIGURATION_REGISTER, 0x09])

    #other calculations before main loop
    gradscale_div = 2**eeprom[E_GRADSCALE]
    vddscgrad_div = 2**eeprom[E_VDDSCGRAD]
    vddscoff_div = 2**eeprom[E_VDDSCOFF]
    pixcij_int32 = np.zeros(shape=(32,32)) #calculate_pixcij() ==> calc sensitivity coefficients (datasheet chapter 11.5)
    for m in range(32):
        for n in range(32):
            pixcij_int32[m][n] = pixcmax - pixcmin
            pixcij_int32[m][n] = pixcij_int32[m][n] / 65535
            pixcij_int32[m][n] = pixcij_int32[m][n] * pij[m][n]
            pixcij_int32[m][n] = pixcij_int32[m][n] * pixcmin
            pixcij_int32[m][n] = pixcij_int32[m][n] * 1.0 * epsilon / 100
            pixcij_int32[m][n] = pixcij_int32[m][n] * 1.0 * globalgain / 10000

    #get timer duration
    timer_duration = calc_timer_duration(bw, clk_calib, mbit_calib)
    print(f"Timer duration: {timer_duration}")

    #check tablenumber
    if tablenumber!=TABLENUMBER:
        print("Connected sensor does not match the selected lookup table.")
        print("The calculated temperatures could be wrong")
        print("Change device in sensordef.py to sensor with tablenumber: ", tablenumber)

    #check buffer length

    print(f"Setup time elapse: {time.time()-start_time}")

    #Start of loop

    #while True: (dont use loop for tesing first)
    #board.i2c_write(SENSOR_ADDRESS, [CONFIGURATION_REGISTER, 0x09])
    time.sleep(timer_duration/(10**6)) #wait for end of conversion bit
    read_sensor(STATUS_REGISTER, 1, cb_statusreg) #read status register
    while statusreg%2==0:
        read_sensor(STATUS_REGISTER, 1, cb_statusreg) #keep reading status register until conversion has finished
    
    read_sensor(TOP_HALF, 258, cb_data_top_block0)
    
    # board.i2c_write(SENSOR_ADDRESS, [CONFIGURATION_REGISTER, 0x19])
    # read_sensor(STATUS_REGISTER, 1, cb_statusreg) #read status register
    # while statusreg%2==0:
    #     read_sensor(STATUS_REGISTER, 1, cb_statusreg)  

    # board.i2c_write(SENSOR_ADDRESS, [CONFIGURATION_REGISTER, 0x29])
    # read_sensor(STATUS_REGISTER, 1, cb_statusreg) #read status register
    # while statusreg%2==0:
    #     read_sensor(STATUS_REGISTER, 1, cb_statusreg)   


    
board = pymata4.Pymata4()
try:
    eeprom = []
    data_top_block0, data_top_block1, data_top_block2, data_top_block3 = [], [], [], []
    data_bottom_block0, data_bottom_block1, data_bottom_block2, data_bottom_block3 = [], [], [], []
    statusreg = 0
    configregister = 0
    main(board)
except KeyboardInterrupt:
    board.shutdown()
finally:
    board.shutdown()