import sys
import time
from math import sqrt, cos, sin, atan2
from Phidget22.Devices.Spatial import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *
from scipy import signal
from scipy import integrate
import numpy as np

try:
    ch = Spatial()
except RuntimeError as e:
    print("Runtime Exception %s" % e.details)
    print("Press Enter to Exit...\n")
    readin = sys.stdin.read(1)
    exit(1)

def AccelerometerAttached(e):
    try:
        attached = e
        print("\nAttach Event Detected (Information Below)")
        print("===========================================")
        print("Library Version: %s" % attached.getLibraryVersion())
        print("Serial Number: %d" % attached.getDeviceSerialNumber())
        print("Channel: %d" % attached.getChannel())
        print("Channel Class: %s" % attached.getChannelClass())
        print("Channel Name: %s" % attached.getChannelName())
        print("Device ID: %d" % attached.getDeviceID())
        print("Device Version: %d" % attached.getDeviceVersion())
        print("Device Name: %s" % attached.getDeviceName())
        print("Device Class: %d" % attached.getDeviceClass())
        print("\n")

    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)

def AccelerometerDetached(e):
    detached = e
    try:
        print("\nDetach event on Port %d Channel %d" % (detached.getHubPort(), detached.getChannel()))
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)

def ErrorEvent(e, eCode, description):
    print("Error %i : %s" % (eCode, description))

i = 0
gx = 0.013171068
gy = 0.026110171
gz = 0.99838277
velocityX = 0
prevDynAccel = 0
prevVelocity = 0
distance = 0
def AccelerationChangeHandler(e, acceleration, a , m, timestamp):
    global velocityX, prevVelocity, prevDynAccel, i, gx, gy, gz,zi, distance
    if i<300:
        gx = (acceleration[0] + gx*i)/(i+1) #cos(pitch)*sin(roll)
        gy = (acceleration[1] + gy*i)/(i+1) #-sin(pitch)
        gz = (acceleration[2] + gz*i)/(i+1)#cos(pitch)*cos(roll)
        i = i+1
        print gx, gy, gz
    else:
        dynAccel = round(acceleration[1] - gx,5)*9.8
        velocityX = integrate.cumtrapz((prevDynAccel,dynAccel), dx = 0.03) + velocityX
        velocityX = velocityX[0]
        distance = integrate.cumtrapz((prevVelocity,velocityX), dx = 0.03) + distance
        prevDynAccel =  dynAccel
        prevVelocity = velocityX
        distance = distance[0]
        print dynAccel, velocityX, distance, timestamp



try:
    ch.setOnAttachHandler(AccelerometerAttached)
    ch.setOnDetachHandler(AccelerometerDetached)
    ch.setOnErrorHandler(ErrorEvent)
    ch.setOnSpatialDataHandler(AccelerationChangeHandler)
    print("Waiting for the Phidget Accelerometer Object to be attached...")
    ch.openWaitForAttachment(5000)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Press Enter to Exit...\n")
    readin = sys.stdin.read(1)
    exit(1)
print("Gathering data for 10 seconds...")

ch.setDataInterval(30)
time.sleep(15)
try:
    ch.close()
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Press Enter to Exit...\n")
    readin = sys.stdin.read(1)
    exit(1)
print("Closed Accelerometer device")
exit(0)
