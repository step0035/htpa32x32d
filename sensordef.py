#Sensor Info
SENSOR_ADDRESS = 0x1A
EEPROM_ADDRESS = 0x50

#I2C Clock
CLOCK_SENSOR = 1200000
CLOCK_EEPROM = 400000

#Timer for sample rate
NORM_BW = 68.598
SAFETY_FAC = 0.90
E_BW1 = 0x0065
E_BW2 = 0x0066

#----EEPROM----
#ID
E_ID1 = 0x0074
E_ID2 = 0x0075
E_ID3 = 0x0076
E_ID4 = 0x0077

#Calibration Settings
E_MBIT_CALIB = 0x001A
E_BIAS_CALIB = 0x001B
E_CLK_CALIB = 0x001C
E_BPA_CALIB = 0x001D
E_PU_CALIB = 0x001E

#User Settings
E_MBIT_USER = 0x0060
E_BIAS_USER = 0x0061
E_CLK_USER = 0x0062
E_BPA_USER = 0x0063
E_PU_USER = 0x0064

#VDD
E_VDDTH1_1 = 0x0026
E_VDDTH1_2 = 0x0027
E_VDDTH2_1 = 0x0028
E_VDDTH2_2 = 0x0029
E_VDDSCGRAD = 0x004E
E_VDDSCOFF = 0x004F

#PTAT TH
E_PTATTH1_1 = 0x003C
E_PTATTH1_2 = 0x003D
E_PTATTH2_1 = 0x003E
E_PTATTH2_2 = 0x003F

#PTAT Gradient
E_PTATGR_1 = 0X0034
E_PTATGR_2 = 0X0035
E_PTATGR_3 = 0X0036
E_PTATGR_4 = 0X0037

#PTAT Offset
E_PTATOFF_1 = 0x0038
E_PTATOFF_2 = 0x0039
E_PTATOFF_3 = 0x003A
E_PTATOFF_4 = 0x003B

#PixCmin/PixCmax
E_PIXCMIN_1 = 0x00000
E_PIXCMIN_2 = 0x00001
E_PIXCMIN_3 = 0x00002
E_PIXCMIN_4 = 0x00003
E_PIXCMAX_1 = 0x00004
E_PIXCMAX_2 = 0x00005
E_PIXCMAX_3 = 0x00006
E_PIXCMAX_4 = 0x00007

#GlobalOff/GlobalGain
E_GLOBALOFF = 0x0054
E_GLOBALGAIN_1 = 0x0055
E_GLOBALGAIN_2 = 0x0056

#Start address for arrays
E_VDDCOMPGRAD = 0x0340
E_VDDCOMPOFF = 0x0540
E_THGRAD = 0x0740
E_THOFFSET = 0x0F40
E_PIJ = 0x1740
E_DEADPIXADR = 0x0080
E_DEADPIXMASK = 0x00B0

#Others
E_NROFDEFPIX = 0x007F
E_GRADSCALE = 0x0008
E_TABLENUMBER1 = 0x000B
E_TABLENUMBER2 = 0x000C
E_EPSILON = 0x000D
E_ARRAYTYPE = 0x0022
E_IP = 0x021C
E_SUBNET = 0x0212
E_MAC = 0x0216

#----Sensor----
#Write Only
CONFIGURATION_REGISTER = 0x01
TRIM_REGISTER1 = 0x03
TRIM_REGISTER2 = 0x04
TRIM_REGISTER3 = 0x05
TRIM_REGISTER4 = 0x06
TRIM_REGISTER5 = 0x07
TRIM_REGISTER6 = 0x08
TRIM_REGISTER7 = 0x09

#Read Only
STATUS_REGISTER = 0x02
TOP_HALF = 0x0A
BOTTOM_HALF = 0x0B

#----Look-up Table----