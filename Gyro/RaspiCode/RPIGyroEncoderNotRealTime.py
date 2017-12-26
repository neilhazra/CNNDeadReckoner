#!/usr/bin/env python3.4
import RPi.GPIO as GPIO
import serial
import logging
import sys
from time import sleep
import time
from math import atan, atan2, sqrt
from thread import start_new_thread
import socket
from Adafruit_BNO055 import BNO055
#from scipy import
#Gyro/Accel
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')
#a = [22, 0, 233, 255, 8, 0, 204, 1, 210, 255, 231, 255, 255, 255, 254, 255, 0, 0, 232, 3, 38, 3]
a = [252, 255, 9, 0, 17, 0, 201, 1, 199, 255, 48, 0, 255, 255, 253, 255, 255, 255, 232, 3, 98, 3]
#bno.set_calibration(a)
sw, bl, accel, mag, gyro = bno.get_revision()

#SsocketComm
dataSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#dataSocket.connect(( '10.0.0.102', 3800)) #rishi laptop
dataSocket.connect(( '10.0.0.61', 3800)) #my laptop
#SerialComm
arduinoSerial = serial.Serial('/dev/ttyACM0', 9600) #Arduino at USB port
arduinoSerial.timeout = None #Read calls should block
arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(0) + ";\n") #Turn Everything Off at beginning
#Encoders
REnc_A = 20
REnc_B = 21
stateR = 0
rightMotorCount = 0
LEnc_A = 10 #TODO
LEnc_B = 11 #TODO
stateL = 0
leftMotorCount = 0
GPIO.setmode(GPIO.BCM)
GPIO.setup(REnc_A, GPIO.IN)
GPIO.setup(REnc_B, GPIO.IN)
GPIO.setup(LEnc_A, GPIO.IN)
GPIO.setup(LEnc_B, GPIO.IN)
def rightEncoder(A_or_B):
    global rightMotorCount, stateR
    s = stateR & 3
    if GPIO.input(REnc_A):
         s = s|4
    if GPIO.input(REnc_B):
         s = s|8
    if s == 0 or s == 5 or s == 10 or s == 15:
        pass
    elif s == 1 or s == 7 or s == 8 or s == 14:
        rightMotorCount = rightMotorCount + 1
    elif s == 2 or s == 4 or s == 11 or s == 13:
        rightMotorCount = rightMotorCount - 1
    elif s == 3 or s == 12:
        rightMotorCount = rightMotorCount + 2
    else:
        rightMotorCount = rightMotorCount - 2
    stateR = s >> 2
    return
def leftEncoder(A_or_B):
    global leftMotorCount, stateL
    s = stateL & 3
    if GPIO.input(LEnc_A):
         s = s|4
    if GPIO.input(LEnc_B):
         s = s|8
    if s == 0 or s == 5 or s == 10 or s == 15:
        pass
    elif s == 1 or s == 7 or s == 8 or s == 14:
        leftMotorCount = leftMotorCount + 1
    elif s == 2 or s == 4 or s == 11 or s == 13:
        leftMotorCount = leftMotorCount - 1
    elif s == 3 or s == 12:
        leftMotorCount = leftMotorCount + 2
    else:
        leftMotorCount = leftMotorCount - 2
    stateL = s >> 2
    return
GPIO.add_event_detect(REnc_A, GPIO.BOTH, callback=rightEncoder)                 # NO bouncetime
GPIO.add_event_detect(REnc_B, GPIO.BOTH, callback=rightEncoder)
GPIO.add_event_detect(LEnc_A, GPIO.BOTH, callback=leftEncoder)              # NO bouncetime
GPIO.add_event_detect(LEnc_B, GPIO.BOTH, callback=leftEncoder)
class OutputData():
    global rightMotorCount, leftMotorCount
    def __init__(self, Voltage): #add more stuff once gyro works
        global rightMotorCount, leftMotorCount
        self.RightEncoder = rightMotorCount/1856.0
        self.LeftEncoder = leftMotorCount/1856.0
        self.Voltage = Voltage
    def update(self, Voltage):
        self.RightEncoder = rightMotorCount/1856.0
        self.LeftEncoder = leftMotorCount/1856.0
        self.Voltage = Voltage
class UserInputData():
    def __init__(self, myTime, leftPower, rightPower):
        self.mytime = myTime
        self.leftPower = leftPower
        self.rightPower = rightPower
inputData = UserInputData(0,-1,-1) #initial values
output = OutputData(0) #initial values

def wifiSocketRelayArduino(name) :
    global inputData
    data = dataSocket.recv(64)
    while not data:
        arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(0) + ";\n")
        pass
    print data
    d = data.split("\n")[0].split(";")
    inputData.mytime = float(d[0])
    inputData.leftPower = float(d[1])
    inputData.rightPower = float(d[2])

def calibrate():
    isDone = False
    while not isDone:
        a,b,c,d = bno.get_calibration_status()
        if a==3 and c==3:
            isDone = True
        print a,b,c,d
calibrate()

while True:
    wifiSocketRelayArduino("SocketArduinoComm") #gets and saves input Data from socket to arduino
    initial = time.time()*1000
    arduinoSerial.write(str(inputData.leftPower) + ";" + str(inputData.rightPower) + ";" + str(inputData.mytime) + ";\n")
    _in = arduinoSerial.readline()
    voltage = float(_in.rstrip("\n"))
    print _in
    print voltage
    prevtime = time.time()
    F = open("/home/pi/Accel/RawData" + time.strftime("%e:%H:%M", time.localtime(time.time())),"w");
    while time.time()*1000-initial<inputData.mytime:
        deltaT = time.time()-prevtime
        prevtime = time.time()
        heading, roll, pitch = bno.read_euler()
        accelx,accely,accelz = bno.read_linear_acceleration()
        #print accelx
        #if abs(accelx) < 0.2:
            #accelx = 0
        F.write(str(accelx)  + "," + str(time.time()-(initial/1000.0)) + "\r\n")
        print bno.get_calibration_status()
        sleep(0.01)
    output.update(voltage)
    s = str(output.RightEncoder) + ";" + str(output.LeftEncoder) + ";" + str(output.Voltage) + ";\n" #create data string
    dataSocket.send(s)
    rightMotorCount = 0
    leftMotorCount = 0
    print "stopped"
    F.close()
    sleep(1);
