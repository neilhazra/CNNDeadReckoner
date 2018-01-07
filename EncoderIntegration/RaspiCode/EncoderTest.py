#!/usr/bin/env python3.4
import RPi.GPIO as GPIO
import serial
import logging
from time import sleep
import time
from math import atan, atan2, sqrt, sin, cos, radians
import socket
from Adafruit_BNO055 import BNO055
import threading
import numpy.matlib
from numpy.linalg import inv
import numpy as np


circumference = 0.2198 #in meters TODO
#Gyro/Accel
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
if status == 0x01:
    print('System error: {0}'.format(error))
#a = [22, 0, 233, 255, 8, 0, 204, 1, 210, 255, 231, 255, 255, 255, 254, 255, 0, 0, 232, 3, 38, 3]
#a = [252, 255, 9, 0, 17, 0, 201, 1, 199, 255, 48, 0, 255, 255, 253, 255, 255, 255, 232, 3, 98, 3]
#bno.set_calibration(a)
sw, bl, accel, mag, gyro = bno.get_revision()
yaw = 0
accelx = 0
accely = 0
#Encoders
clicks_rotation = 1856
REnc_A = 21
REnc_B = 20
stateR = 0
rightMotorCount = 0
LEnc_A = 19
LEnc_B = 26
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
    print rightMotorCount;
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
    print leftMotorCount;
    return

GPIO.add_event_detect(REnc_A, GPIO.BOTH, callback=rightEncoder)                 # NO bouncetime
GPIO.add_event_detect(REnc_B, GPIO.BOTH, callback=rightEncoder)
GPIO.add_event_detect(LEnc_A, GPIO.BOTH, callback=leftEncoder)              # NO bouncetime
GPIO.add_event_detect(LEnc_B, GPIO.BOTH, callback=leftEncoder)
#SsocketComm
dataSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#dataSocket.connect(( '10.0.0.102', 3800)) #rishi laptop
dataSocket.connect(( '10.0.0.61', 3800)) #my laptop
#SerialComm
class OutputData():
    global rightMotorCount, leftMotorCount
    def __init__(self, Voltage): #add more stuff once gyro works
        global rightMotorCount, leftMotorCount
        self.RightEncoder = rightMotorCount
        self.LeftEncoder = leftMotorCount
        self.Voltage = Voltage
        self.Ex = 0
        self.Ey = 0
        self.x1 = 0
        self.y1 = 0
        self.heading = 0
    def update(self, Voltage, Ex,Ey, x1,y1,heading):
        self.RightEncoder = rightMotorCount
        self.LeftEncoder = leftMotorCount
        self.Voltage = Voltage
        self.Ex = Ex
        self.Ey = Ey
        self.x1 = x1
        self.y1 = y1
        self.heading = heading
class UserInputData():
    def __init__(self, myTime, leftPower, rightPower):
        self.mytime = myTime
        self.leftPower = leftPower
        self.rightPower = rightPower
inputData = UserInputData(0,-1,-1) #initial values
output = OutputData(0) #initial values
def ReadIMU():
    global yaw, accelx, accely
    yaw, roll, pitch = bno.read_euler()
    accelx, accely, accelz = bno.read_linear_acceleration()
    accely = -accely
class EncoderIntegration(threading.Thread):
    global rightMotorCount, leftMotorCount, yaw
    def __init__(self):
        global rightMotorCount, leftMotorCount
        threading.Thread.__init__(self)
        #self.F = open("/home/pi/EncoderIntegrationData/EncoderTest" + time.strftime("%e:%H:%M", time.localtime(time.time())),"w");
        self.prevREncoder = 0
        self.prevLEncoder = 0
        self.initialHeading = yaw
        self.x = 0;
        self.y = 0;
        self.velocityx = 0;
        self.velocityy = 0;
        self.heading = 0;
        self.deltaL = 0
        self.deltaR = 0
        self.running = True
    def run(self):
        global yaw
        self.initialHeading = yaw
        prevtime = time.time()
        while self.running:
            dt = time.time() - prevtime
            prevtime  = time.time()
            self.deltaR = (rightMotorCount - self.prevREncoder)*circumference/clicks_rotation
            self.deltaL = (leftMotorCount - self.prevLEncoder)*circumference/clicks_rotation
            self.prevREncoder = rightMotorCount
            self.prevLEncoder = leftMotorCount
            distance = (self.deltaR + self.deltaL)/2
            self.heading = yaw - self.initialHeading
            dx =  distance * sin(radians(self.heading))
            dy =  distance * cos(radians(self.heading))
            self.x = self.x + dx
            self.y = self.y + dy
            self.velocityx = dx/dt
            self.velocityy = dy/dt
            sleep(0.05)
    def stop(self):
        self.running = False
def wifiSocketRelayArduino(name) :
    global inputData
    data = dataSocket.recv(64)

    print data
    d = data.split("\n")[0].split(";")
    inputData.mytime = float(d[0])
    inputData.leftPower = float(d[1])
    inputData.rightPower = float(d[2])
def calibrate():
    isDone = False
    while not isDone:
        a,b,c,d = bno.get_calibration_status()
        if b==3:
            isDone = True
        print a,b,c,d
calibrate()
def kalman(observed, prevState,prevCov,stateTrans,controlMat, controlVec, H ,dynNoiseCov, obsNoiseCov):
    x_predict = np.add(np.matmul(stateTrans, prevState), np.matmul(controlMat,controlVec))
    p_predict = np.add(np.matmul(np.matmul(stateTrans,prevCov),np.transpose(stateTrans)),np.matmul(np.matmul(controlMat,dynNoiseCov),np.transpose(controlMat)))
    yresid = np.subtract(observed,np.matmul(H,x_predict))
    S_k = np.add(obsNoiseCov,np.matmul(np.matmul(H,p_predict),np.transpose(H)))
    K = np.matmul(np.matmul(p_predict,np.transpose(H)),inv(S_k))
    x_update = np.add(x_predict, np.matmul(K,yresid))
    p_update = np.subtract(p_predict,np.matmul(np.matmul(K,H),p_predict))
    return (x_update, p_update)
while True:
    wifiSocketRelayArduino("SocketArduinoComm") #gets and saves input Data from socket to arduino
    initial = time.time()*1000

    E = EncoderIntegration()
    _in = "0\n"
    ReadIMU()
    init_heading = yaw
    E.start()
    voltage = float(_in.rstrip("\n"))
    x_0 = np.asarray([[0],     #Initial State Vector
                      [0],
                      [0],
                      [0]
                    ])
    p_0 = np.asarray([[0,0,0,0],     #initial covariance matrix of state cov(state)
                      [0,0,0,0],
                      [0,0,0,0],
                      [0,0,0,0]
                    ])
    prevtime = time.time()
    while time.time()*1000-initial<inputData.mytime:
        dt = time.time()-prevtime
        prevtime = time.time()
        ReadIMU()
        #Kalman Filter Variables
        F_k = np.asarray([[1,0,dt,0],     #Prediction Matrix
                          [0,1,0,dt],
                          [0,0,1,0],
                          [0,0,0,1]
                        ])
        B_k = np.asarray([[0.5*(dt**2),0,0,0],     #Control Matrix
                          [0,0.5*(dt**2),0,0],
                          [0,0,dt,0],
                          [0,0,0,dt]
                        ])

        rotation_matrix = np.asarray([[cos(radians(yaw-init_heading)),sin(radians(yaw-init_heading))],
                                      [-sin(radians(yaw-init_heading)),cos(radians(yaw-init_heading))]
                                    ])
        raw = np.asarray([[accelx],
                          [accely]
                        ])
        rotated = np.matmul(rotation_matrix,raw)
        #print rotated
        #rotated = np.transpose(rotated)
        u_k = np.asarray([rotated[0],               #control vector, known external influence
                          rotated[1],                #TODO adjust for rotation
                          rotated[0],
                          rotated[1]
                        ])
        #print u_k
        y = np.asarray([[E.x], #x from encoder   #TODO add velocity data from encoder
                        [E.y],  #y from encoder
                        [E.velocityx],
                        [E.velocityy]
                      ])
        H_k = np.asarray([[1, 0, 0, 0],
                          [0, 1, 0, 0],     #observation/state transformation matrix
                          [0, 0, 1, 0],
                          [0, 0, 0, 1],
                        ])
        A = np.asarray(np.diag((0.9,0.9,0.3,0.3))) #dynamic noise
        R = np.asarray(np.diag((0.03,0.03,0.05,0.05)))
        [x1,p1] = kalman(y,x_0,p_0,F_k,B_k, u_k,H_k,A,R)
        x_0 = x1
        p_0 = p1
        print yaw - init_heading
        #print np.transpose(raw)
        #print np.transpose(u_k)
        #print np.transpose(y)
        #print np.transpose(x1)
        sleep(0.0001)
    sleep(2)
    E.stop()
    E.join()
    output.update(voltage,E.x,E.y,x1[0],x1[1],E.heading)
    s = str(output.RightEncoder)+";"+str(output.LeftEncoder)+";"+str(output.Voltage)+";"+str(output.Ex)+";"+str(output.Ey)+";"+str(output.x1)+";"+str(output.y1)+";"+str(output.heading)+";\n" #create data string
    dataSocket.send(s)
    rightMotorCount = 0
    leftMotorCount = 0
    print "stopped"
    sleep(1);
