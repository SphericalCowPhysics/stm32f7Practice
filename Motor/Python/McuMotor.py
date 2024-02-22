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

def toSigned24(n):
    '''
    Converts a 24-bit number from unsigned (range 0~65535) to signed (range -32768~32767). 
    '''
    n = n & 0xffffff
    return (n ^ 0x800000) - 0x800000

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
        'RegEncoderCCwSteps' : 9, 	
    	'RegEncoderPosition'   :10, 
        'EncoderVelocity'       :11,
        'RegLoadCellUpper16' : 12,
        'RegLoadCellLower8' : 13,
        'RegLoadCellOffset' :   14
        }
#regDict['']


##########################################################################
###   Try reading some stuff that is internal to the 'F746
###     
##########################################################################
#ser = serial.Serial(findPort(0x0483,0x5740), timeout=1)
ser = serial.Serial('COM7', baudrate=115200, timeout=1)

#writeReg(regDict['RegEncoderCwSteps'], 0)
#writeReg(regDict['RegEncoderCCwSteps'], 0)  

#print('Firmware Version = ', readReg(regDict['RegFirmWareVersion']))
#print('Unique board ID = ', readReg(regDict['RegUniqueID']))
#print('Timer Tick val = ', readReg(regDict['RegTick'])) 

tempAdc = readReg(regDict['RegAdcTemp'])
#print(f'Temp ADC = {tempAdc}, which is a temperature of {25+(tempAdc*3.3/4095-0.76)/0.025:.2f}°C') 

refAdc = readReg(regDict['RegAdcRef'])
#print(f'internal reference ADC = {refAdc}, which is a voltage of {refAdc*3.3/4095:.3f}V') 

tm1 = readReg(regDict['RegTick'])
time.sleep(1)
tm2 = readReg(regDict['RegTick'])
#print(f'time Difference = {tm2 - tm1} mS')



####################################################################################################################################################
###   Give commands to the motor driver
####################################################################################################################################################
stepperResolutionDict = {       #1.8 degrees per step comes from 360deg/200steps. Each subsequent stepper mode halves this value.
    '000' : 1.8/1,
    '100' : 1.8/2,
    '010' : 1.8/4,
    '110' : 1.8/8,
    '001' : 1.8/16,
    '101' : 1.8/32,
    '011' : 1.8/32,
    '111' : 1.8/32,
    }

modeStepDriverSwitches = '000'      #Each  bit represent voltage Low/High on M0, M1, M2 of step driver board--in that order. These are labeled 2, 3, 4 on dimmer switch.
stepperResolution = stepperResolutionDict[modeStepDriverSwitches]   #degrees per step


encoderResolution = ( 360/600 ) / 4         # = encoder's resolution / number of interrupt triggers per step. Units of degrees/trigger

degreesEncoderClockwisePrevious = readReg(regDict['RegEncoderCwSteps']) * encoderResolution
degreesEncoderCounterClockwisePrevious = readReg(regDict['RegEncoderCCwSteps']) * encoderResolution

###################################################################################################
### BRIEF: Writes to registers in firmware. The firmware has a function in while(1) that makes a step with every call until all steps have been taken.
### steps controls how many steps to take. When M0,1,2 are LOW, 1step=1.8deg 
### duration is how many miliseconds it will take to complete assigned number of steps
### direction determines rotation: CW looking toward load if 1, and CCW if 0
### possible errors: duration should be divisible by steps with no remainder or timing may be off.
###################################################################################################
defaultDelay = 2
def MotorRotateSteps(steps = 200, stepTime = 2, direction = 1, delay = defaultDelay):
    if steps < 0:
        steps = np.abs(steps)
        direction = 0
        #direction = (not direction)
    writeReg(regDict['RegMotorDirEnable'], (0x8000 + (int)(direction)))
    writeReg(regDict['RegMotorStepTime'], (int)(stepTime))
    writeReg(regDict['RegMotorSteps'], (int)(steps))
    time.sleep( (1. + delay) * (steps * stepTime) / 1000 )          #sleep for entire time motor is moving plus delay% longer. Doesn't currently work as intended because sysTick resolution is too coarse
   
   
###################################################################################################
### BRIEF: Determines the current orientation and the fastest way to get to the desired new orientation
###################################################################################################   
#This method needs some optimizing.
#1: !DONE! It should never make multiple revolutions. 
#2: It should determine which direction to rotate (perhaps by determining if its current path would require rotating more than 180*)
#3: Currently converting between steps and degrees and back to steps. Do it once or not at all 
def MotorRotateToNewOrientation(newOrientation, stepTime = 2, delay = defaultDelay):
    currentOrientation = (readReg(regDict['RegEncoderCwSteps']) - readReg(regDict['RegEncoderCCwSteps'])) * encoderResolution
    degreesRequiredToOrient = (newOrientation - currentOrientation) % 360
    if degreesRequiredToOrient < 0:
        direction = 0
        degreesRequiredToOrient = np.abs(degreesRequiredToOrient)
    else:
        direction = 1
        
    if degreesRequiredToOrient > 180:
        degreesRequiredToOrient = (360 - degreesRequiredToOrient)
        
    stepsRequiredToOrient = (int) (degreesRequiredToOrient / stepperResolution)    
    MotorRotateSteps(stepsRequiredToOrient, stepTime, direction, delay)
    
    ''' #
    degreesRequiredToOrient = (newOrientation - currentOrientation) % 360
    if degreesRequiredToOrient > 180:
        degreesRequiredToOrient = 360 - degreesRequiredToOrient
    elif degreesRequiredToOrient < -180:
        degreesRequiredToOrient = 360 - degreesRequiredToOrient
    #stepsRequiredToOrient = (int) (degreesRequiredToOrient / stepperResolution)
    MotorRotateSteps( ((int) (degreesRequiredToOrient / stepperResolution)) , stepTime, delay )
    '''
    
    '''#This first implementation works well, but sometimes takes an unoptimal route (ie rotates > 180deg)
    stepsRequiredToOrient = (int) (  (newOrientation - currentOrientation) % 360)  / stepperResolution  )
    if stepsRequiredToOrient < 0:
        stepsRequiredToOrient = np.abs(stepsRequiredToOrient)
        direction = 0
    else:
        direction - 1
        
    MotorRotateSteps( ((int) (degreesRequiredToOrient / stepperResolution)) , stepTime, delay )
    '''
    
    
#MotorRotateSteps(steps = 1000, stepTime = 1, direction = 1, delay = defaultDelay)    
###################################################################################################
### BRIEF: Converts given signed degrees into steps and direction, then calls on MotorRotateSteps()
###################################################################################################   
def MotorRotateDegrees(degrees = 360, stepTime = 2, direction = 1, delay = defaultDelay):
    #if degrees < 0:
        #degrees = np.abs(degrees)       #If degrees is negative, go that many degrees in the opposite direction as was indicated in argument
        #direction = 0
    steps = (int) (degrees / stepperResolution)
    MotorRotateSteps(steps, stepTime, direction, delay)
 


#stepTime =8 corresponds to about 10mS in reality


#MotorRotateSteps(8*50)
#MotorRotateSteps(8*50)

#MotorRotateDegrees(90)
#MotorRotateDegrees(300)
#MotorRotateToNewOrientation(0)
MotorRotateDegrees(180, 2, 1)
MotorRotateDegrees(600, 1, 0)
MotorRotateDegrees(360, 3, 1)
degreesEncoderClockwise = ( readReg(regDict['RegEncoderCwSteps']) * encoderResolution ) - degreesEncoderClockwisePrevious
degreesEncoderCounterClockwise = ( readReg(regDict['RegEncoderCCwSteps']) * encoderResolution ) - degreesEncoderCounterClockwisePrevious
degreesEncoderTotal = (readReg(regDict['RegEncoderCwSteps']) - readReg(regDict['RegEncoderCCwSteps'])) * encoderResolution    
print(f'The shaft orientation is {degreesEncoderTotal%360:0.1f}°.\nThe shaft rotated {degreesEncoderClockwise:0.1f}° clockwise and {degreesEncoderCounterClockwise:0.1f}° counter-clockwise.')
    



####################################################################################################################################################
###   Read the load cell
####################################################################################################################################################

loadCellAdcToKgConversion = 5 / 8388607
loadCellAdcOffset = 0

def LoadCellWeightMeasurement(units, readout = False):
    loadCellUpper = readReg(regDict['RegLoadCellUpper16'])
    loadCellUpper = loadCellUpper << 8
    loadCellLower = readReg(regDict['RegLoadCellLower8'])
    loadCellLower = loadCellLower >> 8
    
    loadCellAdc = (loadCellUpper | loadCellLower) - loadCellAdcOffset
    if 'adc' == units:
        #loadCellAdc = (loadCellUpper | loadCellLower) - readReg(regDict['RegLoadCellOffset'])
        #loadCellAdc = loadCellAdc - 16674280
        if readout:
            print(f'The load cell reads {loadCellAdc} arbitrary units.')
        return loadCellAdc
    
    elif 'weight' == units:
        #loadCellAdc = (loadCellUpper | loadCellLower) - readReg(regDict['RegLoadCellOffset'])
        loadCellWeightKg = loadCellAdc * loadCellAdcToKgConversion
        if readout:
            print(f'This adc reading converts to {loadCellWeightKg:.2f}kg.')
        return loadCellWeightKg

    else:
        print("Error-- units only accepts 'adc' or 'weight'")


def LoadCellCalibration(samples = 10):
    tempOffset = 0
    for i in np.arange(samples):
        tempOffset += LoadCellWeightMeasurement('adc')
    #writeReg(regDict['RegLoadCellOffset'], (int)(tempOffset / samples)         #Error: the offset can be as high as a 24bit number, but my regs only handle 16
    return (int) (tempOffset / samples)


LoadCellWeightMeasurement('adc', True)
#LoadCellCalibration(10)
#loadCellAdcOffset = readReg(regDict['RegLoadCellOffset'])
loadCellAdcOffset = LoadCellCalibration(10)
LoadCellWeightMeasurement('adc', True)

print("Up force applied")

trials = 20
upForce = 0
for i in np.arange(trials):
    upForce += LoadCellWeightMeasurement('adc', True)
    time.sleep(0.25)
print(f"Load cell averages to {upForce/trials:.2f} when upward force is applied")

downForce = 0
for i in np.arange(trials):
    downForce += LoadCellWeightMeasurement('adc', True)
    time.sleep(0.25)
print(f"Load cell averages to {downForce/trials:.2f} when downward force is applied")

noForce = 0
for i in np.arange(trials):
    noForce += LoadCellWeightMeasurement('adc', True)
    time.sleep(0.25)
print(f"Load cell averages to {noForce/trials:.2f} when no force is applied")































##########################################################################
###   Close the serial connection
###   
##########################################################################

ser.close()
    


