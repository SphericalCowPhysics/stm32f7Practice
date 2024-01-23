# -*- coding: utf-8 -*-
"""
Created on Sun Dec 17 13:40:06 2023

@author: chris
"""
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

regDict = {
        'RegFirmWareVersion' : 0,      
        'RegUniqueID' : 1,                
        'RegTick' : 2,                     
        'RegAdcTemp' : 3,                 
        'RegAdcRef' : 4,                  
        'RegImuWhoAmI' : 5,                
        'RegImuAx' : 6,                   
        'RegImuAy' : 7,                    
        'RegImuAz' : 8,                    
        'RegAdc3TemperatureSense' : 9,           
        'RegAdc3PhotoCurrent' : 10,                
        'RegPWMPulseWidth' : 11,                  
        'RegPWMDutyCycle' : 12,                    
        'RegPWMPeriod' : 13,  }
#regDict['']
##########################################################################
###   Try reading some stuff that is internal to the 'F746
###     
##########################################################################
#ser = serial.Serial(findPort(0x0483,0x5740), timeout=1)
ser = serial.Serial('COM7', baudrate=115200)

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
###   Read & plot some accelerometer data from IMU
###   The MPU-9250 has to be connected on the SPI bus
##########################################################################
testIMU = False
if (testIMU):
    sampleTime = 0.1 #time between samples
    totalTime = 10   #total time to take data
    nSamp = int(totalTime/sampleTime) #number of samples
    
    dat = np.zeros((4,nSamp))
    dat[0,:] = np.linspace(0,totalTime,nSamp)
    
    for i in np.arange(nSamp):
        dat[1,i] = toSigned16(readReg(regDict['RegImuAx']))*2/32767 #convert ADC values to accelerations in g
        dat[2,i] = toSigned16(readReg(regDict['RegImuAy']))*2/32767
        dat[3,i] = toSigned16(readReg(regDict['RegImuAz']))*2/32767
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
###   Read & plot some photoelectric data from photodiode
##########################################################################
testPhotodiode = False
photodiodeIVCurve = False
photodiodeIVCurveTemperatureTests = True

if (testPhotodiode):
    sampleTime = 0.1 #time between samples
    totalTime = 5   #total time to take data
    nSamp = int(totalTime/sampleTime) #number of samples
    pWMPulseStepInterval = int(nSamp / 10)
    
    dat = np.zeros((3,nSamp))   #dat[0] = time, dat[1] = temp, dat[2] = photo-voltage,
    dat[0,:] = np.linspace(0,totalTime,nSamp)
    
    lightSource = 'LED'
    pWMPulseWidth = 0
    pWMPeriod = 1000
    writeReg(regDict['RegPWMPulseWidth'], pWMPulseWidth)
    writeReg(regDict['RegPWMPeriod'], pWMPeriod)
    
    for i in np.arange(nSamp): 
        dat[1,i] = (1 / (1/3650 * -np.log( 4095.00000001 / toSigned16(readReg(regDict['RegAdc3TemperatureSense'])) - 1) + 1 / 298)) - 273        #convert adcValue to celsius. See CUB notebook page 53 for derivation
        dat[2,i] = toSigned16(readReg(regDict['RegAdc3PhotoCurrent'])) * 3.3 / 4095      #convert adcValue to volts. 
        time.sleep(sampleTime)
        
    fig, ax1 = plt.subplots(figsize=(12, 8))
    ax2 = ax1.twinx()
    
    ax1.plot(dat[0,:], dat[2,:], color='g')
    ax1.set_xlabel("Time (S)")
    ax1.set_ylabel("Average Photo-Voltage (V)", color='g')
    ax1.set_ylim(0.0, 2.5)
    averagePhotoVolt = np.sum(dat[2, :]) / nSamp
    ax1.plot(dat[0,:], averagePhotoVolt * np.ones_like(dat[0,:]), color='yellow')
    ax1.annotate(f"Vavg=\n{averagePhotoVolt:.1f}V", (0-0.1, averagePhotoVolt), annotation_clip = False)
    
    ax2.plot(dat[0,:], dat[1,:], color='r')
    ax2.set_ylabel("Temperature (*C)", color='r')
    averageTemp = np.sum(dat[1, :]) / nSamp
    ax2.set_ylim(averageTemp - 1, averageTemp + 1, auto = False)
    ax2.plot(dat[0,:], averageTemp * np.ones_like(dat[0,:]), color='orange')
    ax2.annotate(f"Tavg=\n{averageTemp:.1f}*C", (totalTime-0.05, averageTemp), annotation_clip = False)

    plt.title(f'Photodiode Data\n Duty Cycle: {pWMPulseWidth / pWMPeriod * 100:.0f}%; Light Source: {lightSource}')
 
###########################################################################################################################
if (photodiodeIVCurve):
    themistorTemp = (1 / (1/3650 * -np.log( 4095.00000001 / toSigned16(readReg(regDict['RegAdc3TemperatureSense'])) - 1) + 1 / 298)) - 273
    print(f'Thermistor Temperature = {themistorTemp:0.1f}')
    averagingTimes = 10     #How many values are averaged during a single PW value
    numOfDutyValues = 10    #How many bins Duty values (0-90) are divided into
    datIVCurve = np.zeros((2, numOfDutyValues)) #datIVCurve[0] = current from PWM-OpAMp-Transistor, datIVCurve[1] = averaged voltage
    pWMPulseWidth = 0
    writeReg(regDict['RegPWMPulseWidth'], pWMPulseWidth)    
    pWMPeriod = 1000
    writeReg(regDict['RegPWMPeriod'], pWMPeriod)
    dummyVoltAvg = np.zeros(averagingTimes)

    for i in np.arange(numOfDutyValues):
        for j in np.arange(averagingTimes):
            dummyVoltAvg[j] = toSigned16(readReg(regDict['RegAdc3PhotoCurrent'])) * 3.3 / 4095 

        datIVCurve[0, i] = (pWMPulseWidth / pWMPeriod) * 0.1085
        datIVCurve[1, i] = np.sum(dummyVoltAvg) / averagingTimes
        pWMPulseWidth += int(pWMPeriod / numOfDutyValues)
        writeReg(regDict['RegPWMPulseWidth'], pWMPulseWidth)


    
    plt.figure(figsize=(5,3.5),dpi=150)
    plt.title('Photo Voltage vs PWM-driven Current')
    plt.plot(datIVCurve[0, :], datIVCurve[1, :])
    plt.xlabel('Current (A)')
    plt.ylabel('Voltage (V)')
    plt.xlim(0, 0.11)
    plt.ylim(0, 2.5)
    #plt.annotate(f"Temp=\n{themistorTemp:.1f}*C", (0.09, 2.0), annotation_clip = False)
    #plt.show()
    
###########################################################################################################################
if (photodiodeIVCurveTemperatureTests):

    numOfAveragingTimes = 5000        #How many values are averaged during a single PW value
    numOfDutyValues = 5000        #How many bins Duty values (0.0-1.0) are divided into
    numOfThermTempValues = 7    #currently limited to 7 by length of the colors array
    datIVCurve = np.zeros((1 + numOfThermTempValues, numOfDutyValues)) #datIVCurve[0] = current from PWM-OpAMp-Transistor, datIVCurve[1-7] = averaged voltage
    datThermistorTemp = np.zeros(numOfThermTempValues)
    #haveNotCalculatedCurrent = True

#############   
#Calculates PWM current and calculates an average of photo-voltage at a given duty value, then iterrates until I and V have been calulated for all duty bins.
#The step size between each duty bin is int(pWMPeriod / numOfDutyValues)
############# 
    def IVGraphData(index):
        pWMPulseWidth = 0
        writeReg(regDict['RegPWMPulseWidth'], pWMPulseWidth)    
        pWMPeriod = 1000
        writeReg(regDict['RegPWMPeriod'], pWMPeriod)
        dummyADCVoltAvg = np.zeros(numOfAveragingTimes)         #Temporary array
        dummyADCThermTempAvg = np.zeros(numOfAveragingTimes)    #Temporary array

        for i in np.arange(numOfDutyValues):
            
            for j in np.arange(numOfAveragingTimes):
                dummyADCVoltAvg[j] = toSigned16(readReg(regDict['RegAdc3PhotoCurrent']))    #Temporarily store (numOfAveragingTimes) number of adc voltage values to be converted and averaged outside j loop
            if index == 1:
            #if haveNotCalculatedCurrent:    
                datIVCurve[0, i] = (pWMPulseWidth / pWMPeriod) * 0.1085     #Convert duty to current (A)
                #haveNotCalculatedCurrent = False
            datIVCurve[index, i] = np.sum(dummyADCVoltAvg) / numOfAveragingTimes * 3.3 / 4095   #Convert and average photo-voltage (V)
            dummyADCThermTempAvg[i] = toSigned16(readReg(regDict['RegAdc3TemperatureSense']))   #Temporarily store (numofDutyValues) number of adc thermistor temperature values to be converted and averaged outside the i loop
            pWMPulseWidth += int(pWMPeriod / numOfDutyValues)       #Increase PW value to the value of the next bin
            writeReg(regDict['RegPWMPulseWidth'], pWMPulseWidth)    #Increase timer's pulse width
            
        datThermistorTemp[index - 1] = (1 / (1/3650 * -np.log( 4095.00000001 / (np.sum(dummyADCThermTempAvg) / numOfDutyValues) - 1) + 1 / 298)) - 273      #Convert and average thermistor temperature (*C)
        print(f'Thermistor Temperature = {datThermistorTemp[index - 1]:0.1f}')
#################

    startTemp = 5           #Desired starting temp (*C) for photodiode
    finalTemp = 1 + 65      #Desired starting temp (*C) for photodiode plus a little more to account for temperature fluctuations
    tempInterval = 5        #Temperature interval (*C) between final plots
    #lastTemp = startTemp
    #nextTemp = lastTemp + tempInterval
    nextTemp = startTemp
    currentTemp = (1 / (1/3650 * -np.log( 4095.00000001 / toSigned16(readReg(regDict['RegAdc3TemperatureSense'])) - 1) + 1 / 298)) - 273
    tempCounter = 1
    numThermTempAvgCounterMax = 25
    dummyADCThermTempAvg2 = np.zeros(numThermTempAvgCounterMax)
    while ((currentTemp <= finalTemp) & (tempCounter <= numOfThermTempValues)):     #Redundent checks, while loop ends if the temperature rises too high or we exceed the number of plots we have colors for
        if (currentTemp >= nextTemp):
            IVGraphData(tempCounter)        #Get 
            tempCounter += 1
            #lastTemp, nextTemp = nextTemp, (nextTemp + tempInterval)
            nextTemp += tempInterval
        else:
            time.sleep(0.5)
            for ii in np.arange(numThermTempAvgCounterMax):
                dummyADCThermTempAvg2[ii] = toSigned16(readReg(regDict['RegAdc3TemperatureSense'])) 
            currentTemp = (1 / (1/3650 * -np.log( 4095.00000001 / (np.sum(dummyADCThermTempAvg2) / numThermTempAvgCounterMax) - 1) + 1 / 298)) - 273


    colors = np.array(['b', 'g', 'r', 'c', 'm', 'y', 'k'])
    plt.figure(figsize=(5,3.5),dpi=150)
    plt.title('Photo Voltage vs PWM-driven Current')
    for k in np.arange(numOfThermTempValues):
        plt.plot(datIVCurve[0, :]*1000, datIVCurve[k+1, :], color=colors[k], label=f"{datThermistorTemp[k]:.1f}°C")
    plt.xlabel('Current (mA)')
    plt.ylabel('Voltage (V)')
    plt.xlim(0, 0.11)
    plt.ylim(0, 2.5)
    plt.legend()
    #plt.annotate(f"Temp=\n{themistorTemp:.1f}*C", (0.09, 2.0), annotation_clip = False)
    #plt.show()   
    
##########################################################################
###   Close the serial connection
###   
##########################################################################

ser.close()
