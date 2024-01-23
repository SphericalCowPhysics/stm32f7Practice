# -*- coding: utf-8 -*-
"""
Created on Sun Nov 26 14:42:46 2023

@author: Marc Mignard
ΑΒΓΔΕΖΗΘΙΚΛΜΝΞΟΠΡΣΤΥΦΧΨΩαβγδεζηθικλμνξοπρσςτυφχψωάέήϊίόύϋώΆΈΉΊΌΎΏ±≥≤ΪΫ÷≈°√ⁿ²ˑ
"""

import numpy as np
import matplotlib.pyplot as plt
#pip install pyserial for the serial port stuff (does not work with pip install serial)
import serial
import serial.tools.list_ports
import time
from datetime import datetime

##########################################################################
###   Utility stuff for STM32F746 discovery board with ImuTest01 code
###     
##########################################################################
def findPort(VID,PID,num=1):
    '''
    PID is defined below for each board.
    If there are multiple matching VID:PID, then chose the one given by num
    Using the VID is assigned to ST Micro, so potentially there could be a conflict
    F476 Discovery board:  VID:PID=0483:5740
    '''
    n=0
    name = ''
    for port in serial.tools.list_ports.comports():
        if (port.vid==VID) and (port.pid==PID):
            n=n+1
            if (n==num):
                name = port.name
    return name

def readReg(reg):
    '''
    Read a register on one of Marc's STM32 boards.
    Write a string like "r0"
    Read a string like "r0=1" then parse and return the number at the end
    '''
    ser.write(b'r'+bytes(hex(reg)[2:],'ascii')+b'\n')
    text = ser.readline()
    iStart = 1+text.index(b'=')
    iEnd = text.index(b'\n')
    return int(text[iStart:iEnd],16)
     
def writeReg(reg,val):
    '''
    Write a register on one of Marc's STM32 boards. 
    The format of the string is something like "r12=5a"
    '''
    ser.write(b'w'+bytes(hex(reg)[2:],'ascii')+b'='+bytes(hex(val)[2:],'ascii')+b'\n')
    ser.readline() #need to read response so it doesn't fill up buffer

def toSigned16(n):
    '''
    Converts a 16-bit number from unsigned (range 0~65535) to signed (range -32768~32767). 
    '''
    n = n & 0xffff
    return (n ^ 0x8000) - 0x8000

##########################################################################
###   Try reading some stuff that is internal to the 'F746
###     
##########################################################################

ser = serial.Serial(findPort(0x0483,0x5740), timeout=1)
print(f'Firmware Version = {readReg(0)}')
print(f'Unique board ID = {readReg(1)}')
print(f'Timer Tick val = {readReg(2)}') 

tempAdc = readReg(3)
print(f'Temp ADC = {tempAdc}, which is a temperature of {25+(tempAdc*3.3/4095-0.76)/0.025:.2f}°C') 

refAdc = readReg(4)
print(f'internal reference ADC = {refAdc}, which is a voltage of {refAdc*3.3/4095:.3f}V') 

tm = readReg(2)
time.sleep(1)
print(f'time Difference = {readReg(2) - tm} mS')

##########################################################################
###   Read & plot some accelerometer data from IMU
###   The MPU-9250 has to be connected on the SPI bus
##########################################################################

sampleTime = 0.1 #time between samples
totalTime = 20   #total time to take data
nSamp = int(totalTime/sampleTime) #number of samples

dat = np.zeros((4,nSamp))
dat[0,:] = np.linspace(0,totalTime,nSamp)

for i in np.arange(nSamp):
    dat[1,i] = toSigned16(readReg(6))*2/32767 #convert ADC values to accelerations in g
    dat[2,i] = toSigned16(readReg(7))*2/32767
    dat[3,i] = toSigned16(readReg(8))*2/32767
    time.sleep(sampleTime)

plt.figure(figsize=(5,3.5),dpi=150)
plt.title('accelerometer data')
plt.plot(dat[0,:],dat[1,:],label='X axis')
plt.plot(dat[0,:],dat[2,:],label='Y axis')
plt.plot(dat[0,:],dat[3,:],label='Z axis')
plt.xlabel('time (seconds)')
plt.ylabel('acceleration (g)')
plt.grid(True)
#plt.ylim([i[0],i[-1]])
#plt.xlim([t[0]*scale_ms,t[-1]*scale_ms])
plt.legend()
#plt.savefig('myFile.jpg', bbox_inches='tight')
plt.show()

##########################################################################
###   Close the serial connection
###   
##########################################################################

ser.close()
