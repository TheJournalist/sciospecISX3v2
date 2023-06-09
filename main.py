import math
import os
import struct
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import serial
from PySide6.QtCore import QFile, QIODevice
from PySide6.QtWidgets import QApplication
from colorama import Fore
from colorama import Style
from colorama import init as colorama_init
from ieee754 import IEEE754
from impedance import preprocessing
from prettytable import PrettyTable

from PySide6.QtUiTools import QUiLoader

# Settings
printConsoleTable = True
startFrequency = 1
stopFrequency = int(10 * 1e6)
frequencyCount = 100
precision = 1
amplitude = 0.01
scale = 1  # 0:Linear  1:Log

# Set FrontEnd Settings
measureMode = 0x02  # 4PointMode
channel = 0x01  # BNC
rangeSetting = 0
rangeCodes = [0x01, 0x02, 0x04, 0x06]  # 0x01: 100R    0x02: 10k    0x04: 1M    0x06: 100M
rangeValues = [100, 10000, 1000000, 100000000]

# Measurement
numberOfMeas = 1  # 0: continuos

ztable = PrettyTable(
    ['n', 'frq (Hz)', 'w (rad)', 'Re', 'Im', 'Mag', 'Ph (deg)', 'Ls (H)', 'Cs (F)', 'Rs (Ohm)',
     'Lp (H)', 'Cp (F)', 'Rp (Ohm)', 'Q', 'D', 'Xs', 'Gp', 'Bp'])

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
    port='COM9',
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

ser.flushInput()
ser.flushOutput()


def readSerialData(expected, alternateHex=False):
    while ser.inWaiting() < expected:
        time.sleep(0.01)

    data = ser.read(ser.inWaiting())

    if len(data) == 0:
        return

    print(B + 'sciospec' + RST + ' >> ' + GY + 'ans[' + str(len(data)) + "] > ", end='')
    if alternateHex:
        print(GY + '0x' + ''.join('{:02X}'.format(x) for x in data) + RST)
    else:
        print(GY + str([hex(int(x)) for x in data]) + RST)

    if len(data) > 3:
        if data[0] == 0x18:
            if data[2] == 0x81:
                print(R + 'NACK Not-Acknowledge:' + RST + ' Command has not been executed')
            elif data[2] == 0x82:
                print(R + 'NACK Not-Acknowledge:' + RST + ' Command could not be recognized')
            elif data[2] == 0x83:
                print(G + 'ACK Command-Acknowledge:' + RST + ' Command has been executed successfully')
            elif data[2] == 0x84:
                print(G + 'OK System-Ready Message:' + RST + ' System is operational and ready to receive data')
            elif data[2] == 0x04:
                print(G + 'OK Wake-Up Message:' + RST + ' System boot ready')

            data = data[4:]  # After ACK

    if len(data) == 0:
        return

    # Frequency list
    if data[0] == 0xb7 and data[2] == 0x04:
        frq_list.clear()

        frames = [[]]
        for nby in range(len(data) - 1):
            if data[nby] == 0xb7 and data[nby + 1] == 0x04:
                frames.append([])
            else:
                frames[-1].append(data[nby])

        for k in range(len(frames)):
            for i in range(int(len(frames[k]) / 4) - 1):

                f = struct.unpack('!f', bytes(
                    bytearray([frames[k][3 + i * 4], frames[k][3 + i * 4 + 1], frames[k][3 + i * 4 + 2],
                               frames[k][3 + i * 4 + 3]])))[0]

                if (i + 1) % 64 == 0:
                    continue

                frq_list.append(f)
            # print(f"{f:.04f}")

        print(G + "Frequency List" + RST + " ", end="")
        print(frq_list)

    # Measurement data                0x0e (timestamp enabled)
    if data[0] == 0xb8 and data[1] == 0x0a:
        raw_meas_data = data
        print(G + "Measurement Data" + RST)
        queryTime = time.time() - readSerialData.lastMeasTime
        readSerialData.lastMeasTime = time.time()
        print("Query time: ", queryTime)
        print()

        lspl = [[]]
        for datapointn in range(len(raw_meas_data) - 1):
            if raw_meas_data[datapointn] == 0xb8 and raw_meas_data[datapointn + 1] == 0x0a and datapointn % 13 == 0:
                lspl.append([])
            else:
                lspl[-1].append(raw_meas_data[datapointn])
        lspl.remove([])

        for dataindex in range(len(lspl)):
            id = (lspl[dataindex][1] << 8) + (lspl[dataindex][2])
            # print(id, end=", ")
            # ts = (raw_meas_data[i * 17 + 4] << 24) + (raw_meas_data[i * 17 + 5] << 16) + (
            #        raw_meas_data[i * 17 + 6] << 8) + (raw_meas_data[i * 17 + 7])
            w = 2 * math.pi * frq_list[id]
            re = struct.unpack('!f', bytes(bytearray(
                [lspl[dataindex][3], lspl[dataindex][4], lspl[dataindex][5], lspl[dataindex][6]])))[0]
            im = struct.unpack('!f', bytes(bytearray(
                [lspl[dataindex][7], lspl[dataindex][8], lspl[dataindex][9], lspl[dataindex][10]])))[0]

            mag = math.sqrt(pow(re, 2) + pow(im, 2))
            ph = math.degrees(math.atan2(im, re))
            Rs = re
            Xs = im
            Gp = 1 / re
            Bp = 1 / im
            Cs = -1 / (w * Xs)
            Cp = Bp / w
            Ls = Xs / w
            Lp = -1 / (w * Bp)
            Rp = 1 / Gp
            Q = Xs / Rs
            D = -1 / Q

            ztable.add_row([id, frq_list[id], w, re, im, mag, ph, Ls, Cs, Rs, Lp, Cp, Rp, Q, D, Xs, Gp, Bp])

            with open('data.csv', 'a') as file:
                file.write(str(frq_list[id]) + ", " + str(re) + ", " + str(im) + '\n')

    return data


def sendMsg(desc, msg):
    ser.write(msg)
    print(M + "computer" + RST + ' >> ' + Y + desc + RST + GY + ' - msg[' + str(len(msg)) + '] > ' + str(
        ['0x{:02X}'.format(x) for x in msg]) + RST)


def ftoba(f):
    return bytearray.fromhex(IEEE754(f, 1).str2hex())


def find_nearest(array, value):
    n = [abs(i - value) for i in array]
    idx = n.index(min(n))
    return array[idx]


if __name__ == '__main__':

    '''
    app = QApplication(sys.argv)

    ui_file_name = "designer.ui"
    ui_file = QFile(ui_file_name)
    if not ui_file.open(QIODevice.ReadOnly):
        print(f"Cannot open {ui_file_name}: {ui_file.errorString()}")
        sys.exit(-1)
    loader = QUiLoader()
    window = loader.load(ui_file)
    ui_file.close()
    if not window:
        print(loader.errorString())
        sys.exit(-1)
    window.show()

    sys.exit(app.exec())
    '''

    # Stop Continuos Measurement
    # sendMsg('Stop Measurement', bytes.fromhex('B80100B8'))
    # readAck(1)

    # System RST
    # sendMsg('System RST', bytes.fromhex('A100A1'))
    # readAck(1)

    fig, axes = plt.subplots(nrows=1, ncols=3, squeeze=False, figsize=(5, 9))
    ztable.field_names = ['n', 'frq (Hz)', 'w (rad)', 'Re', 'Im', 'Mag', 'Ph (deg)', 'Ls (H)', 'Cs (F)', 'Rs (Ohm)',
     'Lp (H)', 'Cp (F)', 'Rp (Ohm)', 'Q', 'D', 'Xs', 'Gp', 'Bp']

    # Read ID
    sendMsg('Read ID', bytes.fromhex('D100D1'))
    readSerialData(19, alternateHex=True)

    # Initialize setup
    sendMsg('Initialize setup', bytes.fromhex('B60101B6'))
    readSerialData(4)

    # Settings
    msg = bytes([0xB6, 0x25, 0x03]) + \
          ftoba(startFrequency) + \
          ftoba(stopFrequency) + \
          ftoba(frequencyCount) + \
          bytes([scale]) + \
          ftoba(precision) + \
          ftoba(amplitude) + \
          bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0xB6])
    sendMsg('Settings', msg)
    readSerialData(4)

    # Set Timestamp option         0x01
    msg = bytes([0x97, 0x02, 0x01, 0x00, 0x97])
    sendMsg('Timestamp setting', msg)
    readSerialData(4)

    # Get Frequency List
    sendMsg('Get Frequency List', bytes.fromhex('B70104B7'))
    readSerialData(4 * frequencyCount + 4 + 4 * math.ceil(frequencyCount / 64))

    Zmag = []
    Zph = []

    for rangeSetting in range(4):

        '''
        The device has a stack length of 1 (ISX-3) or 2 (ISX-3 mini or ISX-3 with second channel option or
        InternalMux). So only 1 (or 2) FE settings can be stored. Sending the set FE settings command
        more than 1 (or 2) times results in a NACK return. Send following command to empty the stack:
        B0 03 FF FF FF B0
        '''
        sendMsg('FrontEnd clear', [0xB0, 0x03, 0xFF, 0xFF, 0xFF, 0xB0])
        readSerialData(4)

        # Set FrontEnd Settings
        msg = bytes([0xB0, 0x03, measureMode, channel, rangeCodes[rangeSetting], 0xB0])
        sendMsg('FrontEnd Settings', msg)
        readSerialData(4)

        # Check Sync Time Setting
        # sendMsg('Check Sync Time Setting', bytes.fromhex('BA00BA'))
        # readSerialData(alternateHex=True)

        # Clear sync
        # sendMsg('Clear Sync Time Setting', bytes.fromhex('B90400000000B9'))
        # readSerialData()

        # Store sync
        # sendMsg('Store sync', bytes.fromhex('90 00 90'))
        # readSerialData(2.0)

        # Check Sync Time Setting
        # sendMsg('Check Sync Time Setting', bytes.fromhex('BA00BA'))
        # readSerialData(1.0, alternateHex=True)

        # Start Measurement
        msg = bytes([0xB8, 0x03, 0x01]) + \
              (numberOfMeas).to_bytes(2, byteorder='big') + \
              bytes([0xB8])
        readSerialData.lastMeasTime = time.time()
        sendMsg('Start Measurement', msg)

        # Receive Meas
        if Path('./data.csv').is_file():
            os.remove('./data.csv')

        while not Path('./data.csv').is_file():
            data = readSerialData(len(frq_list) * (2 + 2 + 4 + 4 + 1))

        # Load EIS data
        f, Z = preprocessing.readCSV('./data.csv')
        mag = np.sqrt(np.power(Z.real, 2) + np.power(Z.imag, 2))
        ph = np.rad2deg(np.arctan2(Z.imag, Z.real))

        Zmag.append(mag)
        Zph.append(ph)

        # plt.subplot(1, 3, 1)
        # plt.plot(Z.real, -Z.imag, '*-')
        ## plt.xlim(-10e3, 10e3)
        ## plt.ylim(-10e3, 10e3)
        # plt.grid(True, which="both", ls="-")
        # plt.subplot(1, 3, 2)
        # plt.loglog(f, mag, '*-')
        # plt.grid(True, which="both", ls="-")
        # plt.subplot(1, 3, 3)
        # plt.semilogx(f, ph, '*-')
        # plt.grid(True, which="both", ls="-")
        ## fig.tight_layout(pad=5.0)
        # axes[0][0].set_title('Nyquist plot')
        # axes[0][0].set(xlabel="Z' (Ohm)", ylabel='Z" (Ohm)')
        # axes[0][1].set_title('Impedance Magnitude')
        # axes[0][2].set(xlabel="f (Hz)", ylabel='Mag (Ohm)')
        # axes[0][2].set_title('Impedance Phase')
        # axes[0][2].set(xlabel="f (Hz)", ylabel='Phase (deg)')

        df = pd.DataFrame.from_records(ztable.rows, columns=ztable.field_names)
        pd.DataFrame.to_csv(df, "table.csv", index=False)

        print()
        if printConsoleTable:
            print(ztable)
        ztable.clear()

    '''
    ZmagFinal = []
    ZphFinal = []
    for dat in range(len(frq_list)):
        quo = np.divide([Zmag[0][dat], Zmag[1][dat], Zmag[2][dat], Zmag[3][dat]], rangeValues[:])

        bestFit = np.ndarray.tolist(quo).index(find_nearest(quo, 1))

        ZmagFinal.append(Zmag[bestFit][dat])
        ZphFinal.append(Zph[bestFit][dat])
    '''

    for n in range(4):
        plt.subplot(1, 3, 1)
        plt.grid(True, which="both", ls="-")
        plt.subplot(1, 3, 2)
        plt.loglog(frq_list, Zmag[n][:], '*-')
        plt.grid(True, which="both", ls="-")
        plt.subplot(1, 3, 3)
        plt.semilogx(frq_list, Zph[n][:], '*-')
        plt.grid(True, which="both", ls="-")
        # fig.tight_layout(pad=5.0)
        axes[0][0].set_title('Nyquist plot')
        axes[0][0].set(xlabel="Z' (Ohm)", ylabel='Z" (Ohm)')
        axes[0][1].set_title('Impedance Magnitude')
        axes[0][2].set(xlabel="f (Hz)", ylabel='Mag (Ohm)')
        axes[0][2].set_title('Impedance Phase')
        axes[0][2].set(xlabel="f (Hz)", ylabel='Phase (deg)')

    ser.flushInput()
    ser.flushOutput()

    #mng = plt.get_current_fig_manager()
    #mng.full_screen_toggle()
    plt.show()
    plt.pause(0.05)
