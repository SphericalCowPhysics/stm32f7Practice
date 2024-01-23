# -*- coding: utf-8 -*-
"""
Created on Fri Jan 19 09:06:42 2024

@author: MarcMignard
ΑΒΓΔΕΖΗΘΙΚΛΜΝΞΟΠΡΣΤΥΦΧΨΩαβγδεζηθικλμνξοπρσςτυφχψωάέήϊίόύϋώΆΈΉΊΌΎΏ±≥≤ΪΫ÷≈°√ⁿ²ˑ
"""

import numpy as np
import matplotlib.pyplot as plt
import pyvisa
import time
#import datetime
from datetime import datetime
#have to install https://www.ni.com/en-us/support/downloads/drivers/download.ni-visa.html
#then 'pip install pyvisa'

rm = pyvisa.ResourceManager()
rm.list_resources()
inst = rm.open_resource('USB0::0x2EC7::0x8800::800872011777870004::INSTR') #chose the device that looks like this, but that is listed from the line above
#inst = rm.open_resource('USB0::0x2EC7::0x9200::800888011777520056::INSTR') #chose the device that looks like this, but that is listed from the line above
print(inst.query("*IDN?")) #B&K Precision., 8600B, 800872011777870004, 1.39-1.42
inst.close()

inst.write('INPut ON')

curr = np.linspace(0,0.05,100)
vol = np.zeros(curr.size)
dat = np.concatenate((curr,vol)).reshape(2,-1)
for c in np.arange(curr.size):
    inst.write(f':CURRent {curr[c]}')
    time.sleep(0.1)
    resp = inst.query(':MEASure:VOLTage?')
    dat[1,c] = float(resp.strip())
    print(f'current = {curr[c]:.2f}, voltage = {resp}')

inst.write(':CURRent 0')
print('done')
fileName = datetime.now().strftime('.//data//Solar_%Y%m%d_%H%M%S.csv')
np.savetxt(fileName,dat,delimiter=',')

#dat6339_49 = np.genfromtxt('.//data//DcrMeterAdcTest231031_idx1519_6399_49.txt', dtype=float) #, delimiter=',') #, skip_header=3)

plt.figure(figsize=(5,3.5),dpi=150)
plt.title('solar cell current/voltage plot')
plt.plot(dat[0,:],dat[1,:])
plt.xlabel('current')
plt.ylabel('voltage')
plt.grid(True)
#plt.ylim([i[0],i[-1]])
#plt.xlim([t[0]*scale_ms,t[-1]*scale_ms])
#plt.legend()
#plt.savefig('myFile.svg', bbox_inches='tight')
plt.show()

fns = np.array([
    [10,'Solar_20231118_150946.csv'],
    [20,'Solar_20231118_151043.csv'],
    [25,'Solar_20231118_151147.csv'],
    [35,'Solar_20231118_152715.csv'],
    [40,'Solar_20231118_152219.csv'],
    [45,'Solar_20231118_152055.csv'],
    [60,'Solar_20231118_151926.csv']])

plt.figure(figsize=(5,3.5),dpi=150)
plt.title('solar cell current/voltage plot')
for t in np.arange(fns.shape[0]):
    dat = np.genfromtxt(f'.//data//{fns[t,1]}', dtype=float, delimiter=',')
    plt.plot(1000*dat[0,:],dat[1,:],label=f'T={fns[t,0]}°C')
plt.xlabel('current (mA)')
plt.ylabel('voltage (V)')
plt.grid(True)
plt.ylim([0,2.5])
plt.xlim([1000*dat[0,0],1000*dat[0,-1]])
plt.legend()
#plt.savefig('myFile.svg', bbox_inches='tight')
plt.show()

plt.figure(figsize=(5,3.5),dpi=150)
plt.title('solar cell current/power plot')
for t in np.arange(fns.shape[0]):
    dat = np.genfromtxt(f'.//data//{fns[t,1]}', dtype=float, delimiter=',')
    plt.plot(1000*dat[0,:],1000*dat[1,:]*dat[0,:],label=f'T={fns[t,0]}°C')
plt.xlabel('current (mA)')
plt.ylabel('power (mW)')
plt.grid(True)
plt.ylim([0,70])
plt.xlim([1000*dat[0,0],1000*dat[0,-1]])
plt.legend()
#plt.savefig('myFile.svg', bbox_inches='tight')
plt.show()

area = 0.045**2      #solar panel area in m^2
maxPower = 70e-3    #maximum power observed
irrad = 360         #measured irradiance (W/m^2)
powerPerArea = maxPower/area
efficiency = maxPower/area/irrad
