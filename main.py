import struct
import time
import serial
from ieee754 import IEEE754
from colorama import init as colorama_init
from colorama import Fore
from colorama import Style

R = Fore.RED
G = Fore.GREEN
B = Fore.BLUE
M = Fore.MAGENTA
Y = Fore.YELLOW
GY = Fore.LIGHTBLACK_EX
RST = Style.RESET_ALL


colorama_init()

ser = serial.Serial(
        port='COM6',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=None,
        xonxoff=False,
        rtscts=False,
        write_timeout=None,
        dsrdtr=False,
        inter_byte_timeout=None,
        exclusive=None)

def readAck(t = 1):
    timeout = time.time() + t
    data = []
    while ser.inWaiting() or time.time() - timeout < 0.0:
        if ser.inWaiting() > 0:
            data += ser.read(ser.inWaiting())
            timeout = time.time() + t

    print(B + 'sciospec' + RST +' >> ', end='')
    if len(data) > 3:
        if data[2] == 0x81:
            print(R + 'NACK Not-Acknowledge:' + RST + ' Command has not been executed')
        elif data[2] == 0x82:
            print(R + 'NACK Not-Acknowledge:' + RST +  ' Command could not be recognized')
        elif data[2] == 0x83:
            print(G + 'ACK Command-Acknowledge:' + RST +  ' Command has been executed successfully')
        elif data[2] == 0x84:
            print(G + 'OK System-Ready Message:' + RST +  ' System is operational and ready to receive data')
        elif data[2] == 0x04:
            print(G + 'OK Wake-Up Message:' + RST +  ' System boot ready')

def readSerialData(t = 1.0, alternateHex=False):
    timeout = time.time() + t
    data = []
    while ser.inWaiting() or time.time() - timeout < 0.0:
        if ser.inWaiting() > 0:
            data += ser.read(ser.inWaiting())
            timeout = time.time() + t
    print(B + 'sciospec' + RST + ' >> ', end='')
    if(alternateHex):
        print('0x' + ''.join('{:02X}'.format(x) for x in data))
    else:
        print(str([hex(int(x)) for x in data]))
    return data

def sendMsg(desc, msg):
    ser.write(msg)
    print(M + "computer" + RST + ' >> ' + Y + desc + RST + GY + ' - msg[' + str(len(msg)) + '] > ' + str(['0x{:02X}'.format(x) for x in msg]) + RST)

def ftoba(f):
    return bytearray.fromhex(IEEE754(f, 1).str2hex())


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    # Read ID
    sendMsg('Read ID', bytes.fromhex('D100D1'))
    readSerialData(alternateHex=True)

    # Initialize setup
    sendMsg('Initialize setup', bytes.fromhex('B60101B6'))
    readAck()

    # Settings
    startFrequency = 500
    stopFrequency = int(5e6)
    frequencyCount = 80
    precision = 1
    amplitude = 1
    scale = 1

    msg = bytes([0xB6, 0x16, 0x03]) + \
          ftoba(startFrequency) + \
          ftoba(stopFrequency) + \
          ftoba(frequencyCount) + \
          bytes([scale]) + \
          ftoba(precision) + \
          ftoba(amplitude) + \
          bytes([0xB6])
    sendMsg('Settings', msg)
    readAck()

    # Set FrontEnd Settings
    measureMode = 0x02  # 4PointMode
    channel = 0x02  # BNC
    rangeSetting = 0x03  # 100k
    msg = bytes([0xB0, 0x03, measureMode, channel, rangeSetting, 0xB0])
    sendMsg('FrontEnd Settings', msg)
    readAck()

    # Start Measurement
    numberOfMeas = 1
    msg = bytes([0xB8, 0x03, 0x01]) + \
          numberOfMeas.to_bytes(2, byteorder='big') + \
          bytes([0xB8])
    sendMsg('Start Measurement', msg)
    readAck()

    # Receive Meas
    while True:
        data = readSerialData(5)
