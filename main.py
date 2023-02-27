import math
import os
import struct
import time

import numpy as np
import pandas as pd
import serial
from prettytable import PrettyTable
from ieee754 import IEEE754
from colorama import init as colorama_init
from colorama import Fore
from colorama import Style
from pathlib import Path
import matplotlib.pyplot as plt
from impedance.visualization import plot_nyquist,  plot_bode
from impedance import preprocessing


# Settings
startFrequency = 1000
stopFrequency = int(2e6)
frequencyCount = 50
precision = 2
amplitude = 0.25
scale = 1               # 0:Linear  1:Log

# Set FrontEnd Settings
measureMode = 0x02      # 4PointMode
channel = 0x01          # BNC
rangeSetting = 0x01     # 0x01: 100R    0x02: 10k    0x04: 1M

# Measurement
numberOfMeas = 1        # 0: continuos


ztable = PrettyTable(['n', 'timestamp (s)', 'frq (Hz)', 'w (rad)', 'Re', 'Im', 'Mag', 'Ph (deg)', 'Ls (H)','Cs (F)','Rs (Ohm)','Lp (H)','Cp (F)','Rp (Ohm)','Q','D','Xs','Gp','Bp'])

R = Fore.RED
G = Fore.GREEN
B = Fore.BLUE
M = Fore.MAGENTA
Y = Fore.YELLOW
GY = Fore.LIGHTBLACK_EX
RST = Style.RESET_ALL

frq_list = []
colorama_init()

ser = serial.Serial(
        port='COM6',
        baudrate=2000000,
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

def readAck(t = 0.1):
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
    print('')

def readSerialData(t = 0.1, alternateHex=False):
    timeout = time.time() + t
    data = []
    while ser.inWaiting() or time.time() - timeout < 0.0:
        if ser.inWaiting() > 0:
            data += ser.read(ser.inWaiting())
            timeout = time.time() + t

    if len(data) == 0:
        return

    print(B + 'sciospec' + RST + ' >> ' + GY + 'ans[' + str(len(data)) + "] > ", end='')
    if(alternateHex):
        print(GY + '0x' + ''.join('{:02X}'.format(x) for x in data) + RST + '\r\n')
    else:
        print(GY + str([hex(int(x)) for x in data]) + RST+ '\r\n')

    # Frequency list
    if data[0] == 0xb7 and data[2] == 0x04:
        frq_list.clear()
        for i in range(int(len(data)/4) - 2):
            f = struct.unpack('!f', bytes(bytearray([data[3 + i*4], data[3 + i*4 + 1], data[3 + i*4 + 2], data[3 + i*4 + 3]])))[0]
            frq_list.append(f)
            #print(f"{f:.04f}")

        print(G + "Frequency List" + RST)
        print(frq_list, '\r\n')

    # Measurement data
    if data[0] == 0xb8 and data[1] == 0x0e:
        raw_meas_data = data
        print(G + "Measurement Data" + RST)
        queryTime = time.time() - readSerialData.lastMeasTime
        readSerialData.lastMeasTime = time.time()
        print("Query time: ", queryTime)
        print()

        for i in range(int(len(raw_meas_data)/17)):
            id = (raw_meas_data[i*17+2]<<8) + (raw_meas_data[i*17+3])
            ts = (raw_meas_data[i * 17 + 4] << 24) + (raw_meas_data[i * 17 + 5] << 16) + (raw_meas_data[i * 17 + 6] << 8) + (raw_meas_data[i * 17 + 7])
            w = 2 * math.pi * frq_list[id]
            re = struct.unpack('!f', bytes(bytearray([raw_meas_data[i*17+8],raw_meas_data[i*17+9],raw_meas_data[i*17+10],raw_meas_data[i*17+11]])))[0]
            im = struct.unpack('!f', bytes(bytearray([raw_meas_data[i * 17 + 12], raw_meas_data[i * 17 + 13], raw_meas_data[i * 17 + 14],raw_meas_data[i * 17 + 15]])))[0]
            mag = math.sqrt(pow(re,2) + pow(im,2))
            ph = math.degrees(math.atan2(im,re))
            Rs = re
            Xs = im
            Gp = 1/re
            Bp = 1/im
            Cs = -1/(w*Xs)
            Cp = Bp/w
            Ls = Xs / w
            Lp = -1 / (w * Bp)
            Rp = 1 / Gp
            Q = Xs / Rs
            D = -1 / Q

            ztable.add_row([id,ts,frq_list[id],w,re,im,mag,ph,Ls,Cs,Rs,Lp,Cp,Rp,Q,D,Xs,Gp,Bp])

            with open('data.csv', 'a') as file:
                file.write(str(frq_list[id]) + ", " + str(re) + ", " + str(im) + '\n')
        print(ztable)

    return data

def sendMsg(desc, msg):
    ser.write(msg)
    print(M + "computer" + RST + ' >> ' + Y + desc + RST + GY + ' - msg[' + str(len(msg)) + '] > ' + str(['0x{:02X}'.format(x) for x in msg]) + RST)

def ftoba(f):
    return bytearray.fromhex(IEEE754(f, 1).str2hex())


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    # Stop Continuos Measurement
    #sendMsg('Stop Measurement', bytes.fromhex('B80100B8'))
    #readAck(1)

    # System RST
    #sendMsg('System RST', bytes.fromhex('A100A1'))
    #readAck(1)

    # Read ID
    sendMsg('Read ID', bytes.fromhex('D100D1'))
    readSerialData(alternateHex=True)

    # Initialize setup
    sendMsg('Initialize setup', bytes.fromhex('B60101B6'))
    readAck()

    # Settings
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

    # Get Frequency List
    sendMsg('Get Frequency List', bytes.fromhex('B70104B7'))
    readSerialData()

    # Set Timestamp option
    msg = bytes([0x97, 0x02, 0x01, 0x01, 0x97])
    sendMsg('Timestamp setting', msg)
    readAck()

    # Set FrontEnd Settings
    msg = bytes([0xB0, 0x03, measureMode, channel, rangeSetting, 0xB0])

    '''
    The device has a stack length of 1 (ISX-3) or 2 (ISX-3 mini or ISX-3 with second channel option or
    InternalMux). So only 1 (or 2) FE settings can be stored. Sending the set FE settings command
    more than 1 (or 2) times results in a NACK return. Send following command to empty the stack:
    B0 03 FF FF FF B0
    '''
    sendMsg('FrontEnd clear', [0xB0, 0x03, 0xFF, 0xFF, 0xFF, 0xB0])
    readAck()

    sendMsg('FrontEnd Settings', msg)
    readAck()

    # Start Measurement
    msg = bytes([0xB8, 0x03, 0x01]) + \
          (numberOfMeas + 1).to_bytes(2, byteorder='big') + \
          bytes([0xB8])
    readSerialData.lastMeasTime = time.time()
    sendMsg('Start Measurement', msg)
    readAck(1)

    # Receive Meas

    for i in range(numberOfMeas):

        if Path('./data.csv').is_file():
            os.remove('./data.csv')

        while not Path('./data.csv').is_file():
            data = readSerialData()

        # Load EIS data
        f, Z = preprocessing.readCSV('./data.csv')
        f, Z = preprocessing.ignoreBelowX(f, Z)

        mag = np.sqrt(np.power(Z.real, 2) + np.power(Z.imag, 2))
        ph = np.rad2deg(np.arctan2(Z.imag, Z.real))

        fig, axes = plt.subplots(nrows=3, ncols=1, squeeze=False, figsize=(5, 9))
        plt.subplot(311)
        plt.plot(Z.real, -Z.imag, '*r-')
        plt.grid(True, which="both", ls="-")
        plt.subplot(312)
        plt.semilogx(f, mag, '*r-')
        plt.grid(True, which="both", ls="-")
        plt.subplot(313)
        plt.semilogx(f, ph, '*r-')
        plt.grid(True, which="both", ls="-")
        fig.tight_layout(pad=5.0)
        axes[0][0].set_title('Nyquist plot')
        axes[0][0].set(xlabel="Z' (Ohm)", ylabel='Z" (Ohm)')
        axes[1][0].set_title('Impedance Magnitude')
        axes[1][0].set(xlabel="f (Hz)", ylabel='Mag (Ohm)')
        axes[2][0].set_title('Impedance Phase')
        axes[2][0].set(xlabel="f (Hz)", ylabel='Phase (deg)')
        df = pd.DataFrame.from_records(ztable.rows, columns=ztable.field_names)
        plt.show(block=True)
