#!/usr/bin/env python3.4
import RPi.GPIO as GPIO
import serial
import os
import logging
from time import sleep
import time
from math import atan, atan2, sqrt, sin, cos, radians, degrees
import socket
from Adafruit_BNO055 import BNO055
import threading
import numpy.matlib
from numpy.linalg import inv
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
circumferenceL = 0.226 #in meters TODO
circumferenceR = 0.226
clicks_rotation = 1920
width = 0.164
#Gyro/Accel
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
yaw = 0
accelx = 0
accely = 0
rightMotorCount = 0
leftMotorCount = 0
#SsocketComm
dataSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#dataSocket.connect(( '10.0.0.102', 3800)) #rishi laptop
dataSocket.connect(( '10.0.0.61', 3800)) #my laptop
#SerialComm
arduinoSerial = serial.Serial('/dev/ttyACM0', 9600) #Arduino at USB port
arduinoSerial.timeout = None #Read calls should block
arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(0) + ";\n") #Turn Everything Off at beginning
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
#class ReadArduino(threading.Thread):
    global rightMotorCount, leftMotorCount
    def __init__(self):
        arduinoSerial.timeout = 0.1
        self.running = True
        threading.Thread.__init__(self)
    def run(self):
        global rightMotorCount, leftMotorCount
        while self.running:
            sleep(0.01)
            _in = arduinoSerial.readline() #block and read newest (very fast ideally)
            arduinoSerial.flushInput()
            print _in
            d = _in.split("\n")[0].split(";")
            try:
                leftMotorCount = int(d[0])
                rightMotorCount = int(d[1])
            except ValueError:
                pass;
    def stop(self):
        arduinoSerial.timeout = None
        self.running = False
class EncoderIntegration(threading.Thread):
    global rightMotorCount, leftMotorCount
    def __init__(self):
        global rightMotorCount, leftMotorCount, yaw
        threading.Thread.__init__(self)
        arduinoSerial.timeout = 0.01
        self.F = open("/home/pi/MLRobot/Data/Enc_" + time.strftime("%e:%H:%M", time.localtime(time.time())),"w");
        self.prevREncoder = 0
        self.prevLEncoder = 0
        self.prevHeading = 0
        self.x = 0;
        self.y = 0
        self.velocityx = 0
        self.gyroInitialAngle = yaw
        self.gyroAngle = yaw
        self.velocityy = 0
        self.heading = 0
        self.deltaL = 0
        self.deltaR = 0
        self.running = True
        self.initialTime = time.time()*1000
    def run(self):
        global leftMotorCount, rightMotorCount, yaw
        prevtime = time.time()
        x = []
        y = []
        line = "deltaR,deltaL,prevx,prevy,dx,dy,x,y,degrees,gyro\n"
        self.F.write(line);
        self.gyroInitialAngle = yaw
        while self.running:
            self.gyroAngle = yaw - self.gyroInitialAngle
            _in = arduinoSerial.readline() #block and read newest (very fast ideally)
            print str(_in) + " " + str(time.time()*1000-self.initialTime)
            d = _in.split("\n")[0].split(";")
            try:
                leftMotorCount = int(d[0])
                rightMotorCount = int(d[1])
            except ValueError:
                pass;
            dt = time.time() - prevtime
            prevtime  = time.time()
            self.deltaR = (rightMotorCount - self.prevREncoder)*circumferenceR/clicks_rotation
            self.deltaL = (leftMotorCount - self.prevLEncoder)*circumferenceL/clicks_rotation
            self.prevREncoder = rightMotorCount
            self.prevLEncoder = leftMotorCount
            deltaTheta = atan((self.deltaL-self.deltaR)/width)
            self.heading = self.prevHeading + deltaTheta #radians
            self.prevHeading = self.heading
            distance = (self.deltaR + self.deltaL)/2
            dx =  distance * sin(self.prevHeading+deltaTheta/2)
            dy =  distance * cos(self.heading)
            line = str(self.deltaR) + ","+ str(self.deltaL)+ "," + str(self.x) + ","+ str(self.y) + ","+ str(dx) + "," + str(dy)+ "," + str(float(self.x + dx)) + "," + str(float(self.y + dy))+ "," + str(degrees(self.heading)) + "," + str(self.gyroAngle) + "\n"
            self.x = self.x + dx
            self.y = self.y + dy
            self.velocityx = dx/dt
            self.velocityy = dy/dt
            x.append(self.x)
            y.append(self.y)
            try:
                self.F.write(line);
            except ValueError:
                pass
        plt.plot(y,x,'bo')
        #pyplot.axis('scaled')
        plt.axis([0,max(y),max(abs(max(x)),abs(min(x))),-max(abs(max(x)),abs(min(x)))])
        plt.savefig(os.path.join("/home/pi/MLRobot/Data", time.strftime("%e:%H:%M", time.localtime(time.time()))+"graph.png"),bbox_inches='tight',dpi= 300)
        plt.close()
    def stop(self):
        self.running = False
        self.F.close()
        arduinoSerial.timeout = None
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
    print inputData.leftPower, inputData.rightPower
    arduinoSerial.write(str(inputData.leftPower) + ";" + str(inputData.rightPower) + ";" + str(inputData.mytime) + ";\n")
    _in = arduinoSerial.readline()
    E = EncoderIntegration()
    #Reader = ReadArduino()
    #Reader.start()
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
        #print np.transpose(raw)
        #print np.transpose(u_k)
        #print np.transpose(y)
        #print np.transpose(x1)
        #sleep(0.0001)
    sleep(0.5)
    E.stop()
    #Reader.stop()
    E.join()
    #Reader.join()
    output.update(voltage,E.x,E.y,x1[0],x1[1],E.heading)
    tempX = (E.x+0.065*sin(radians(E.heading)))
    tempY = (E.y+0.065*(cos(radians(E.heading))-1))
    s = str(output.RightEncoder)+";"+str(output.LeftEncoder)+";"+str(output.Voltage)+";"+str(output.Ex)+";"+str(output.Ey)+";"+str(tempX)+";"+str(tempY)+";"+str(output.heading)+";\n" #create data string
    dataSocket.send(s)
    rightMotorCount = 0
    leftMotorCount = 0
    print "stopped"
    sleep(1);
