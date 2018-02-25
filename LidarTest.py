#!/usr/bin/env python3.4
import smbus
from time import sleep
import time
bus = smbus.SMBus(1)
sleep(0.05)
address = 0x62
distWriteReg = 0x00
distWriteVal = 0x04
distReadReg1 = 0x8f
distReadReg2 = 0x10
velWriteReg = 0x04
velWriteVal = 0x08
velReadReg = 0x09
def writeAndWait(register, value):
    bus.write_byte_data(address, register, value);
    time.sleep(0.02)

def readAndWait(register):
    res = bus.read_byte_data(address, register)
    time.sleep(0.02)
    return res
def getDistance():
    writeAndWait(distWriteReg, distWriteVal)
    dist1 = readAndWait(distReadReg1)
    dist2 = readAndWait(distReadReg2)
    return (dist1 << 8) + dist2

while True:
    #bus.write_byte_data(0x62,0x00,0x04)
    sleep(0.02)
    print( getDistance())
