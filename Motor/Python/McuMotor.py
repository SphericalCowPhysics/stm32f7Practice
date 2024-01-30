# -*- coding: utf-8 -*-
"""
Created on Thu Jan 25 15:50:11 2024

@author: chris
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

regDict = {
        'RegFirmWareVersion' : 0,      
        'RegUniqueID' : 1,                
        'RegTick' : 2,                     
        'RegAdcTemp' : 3,                 
        'RegAdcRef' : 4,
        'RegMotorDirEnable' : 5,                     
        'RegMotorSteps' : 6,                 
        'RegMotorStepTime' : 7,
        'RegEncoderCwSteps' : 8,
        'RegEncoderCCwSteps' : 9
        }
#regDict['']
##########################################################################
###   Try reading some stuff that is internal to the 'F746
###     
##########################################################################
#ser = serial.Serial(findPort(0x0483,0x5740), timeout=1)
ser = serial.Serial('COM7', baudrate=115200, timeout=1)

print('Firmware Version = ', readReg(regDict['RegFirmWareVersion']))
print('Unique board ID = ', readReg(regDict['RegUniqueID']))
print('Timer Tick val = ', readReg(regDict['RegTick'])) 

tempAdc = readReg(regDict['RegAdcTemp'])
print(f'Temp ADC = {tempAdc}, which is a temperature of {25+(tempAdc*3.3/4095-0.76)/0.025:.2f}°C') 

refAdc = readReg(regDict['RegAdcRef'])
print(f'internal reference ADC = {refAdc}, which is a voltage of {refAdc*3.3/4095:.3f}V') 

tm1 = readReg(regDict['RegTick'])
time.sleep(1)
tm2 = readReg(regDict['RegTick'])
print(f'time Difference = {tm2 - tm1} mS')



##########################################################################
###   Give commands to the motor driver
##########################################################################

### steps controls how many steps to take. When M0,1,2 are LOW, 1step=1.8deg 
### duration is how many miliseconds it will take to complete assigned number of steps
### direction determines rotation: CW looking toward load if 1, and CCW if 0
### possible errors: duration should be divisible by steps with no remainder or timing may be off.
def RotateMotor(steps = 200, stepTime = 2, direction = 1, delay = 0.5):
   writeReg(regDict['RegMotorDirEnable'], (0x8000 + direction))
   writeReg(regDict['RegMotorStepTime'], stepTime)
   writeReg(regDict['RegMotorSteps'], steps)
   time.sleep( (1. + delay) * (steps * stepTime) / 1000 )          #sleep for entire time motor is moving plus delay% longer
   
   
#stepTime =8 corresponds to about 10mS in reality
RotateMotor(10, 2, 0)
print('RegMotorDirEnable = ', readReg(regDict['RegMotorDirEnable']))
print('RegMotorSteps = ', readReg(regDict['RegMotorSteps']))
print('RegMotorStepTime = ', readReg(regDict['RegMotorStepTime']))
print('111111111111111')





encoderResolution = 0.6
stepsEncoderClockwise = readReg(regDict['RegEncoderCwSteps'])
stepsEncoderCounterClockwise = readReg(regDict['RegEncoderCCwSteps'])
stepsEncoderTotal = stepsEncoderClockwise - stepsEncoderCounterClockwise
degreesEncoderTotal = stepsEncoderTotal * encoderResolution
print(f'The shaft rotated {degreesEncoderTotal:0.1f}° clockwise')

 
##########################################################################
###   Close the serial connection
###   
##########################################################################

ser.close()
    


