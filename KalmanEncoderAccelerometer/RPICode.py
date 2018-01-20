#!/usr/bin/env python3.4
import RPi.GPIO as GPIO
import serial
import os
import logging
from time import sleep
import time
from math import atan, atan2, sqrt, sin, cos, radians, degrees, acos
import socket
from Adafruit_BNO055 import BNO055
import threading
import numpy.matlib
from numpy.linalg import inv
from numpy import linalg as LA
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from Phidget22.Devices.Spatial import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *
#TRIAL NUMBER
trialNumber = 1
#Constantes
circumferenceL = 0.226
circumferenceR = 0.226
clicks_rotation = 1920
width = 0.164
#Gyro/Accel
samplingTime = 5
#encoders
rightMotorCount = 0
leftMotorCount = 0
#SsocketComm
dataSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#dataSocket.connect(( '10.0.0.102', 3800)) #rishi laptop
dataSocket.connect(( '10.0.0.61', 3800)) #my laptop
#dataSocket.connect(( '10.0.0.163', 3800))
#SerialComm
arduinoSerial = serial.Serial('/dev/ttyACM0', 9600) #Arduino at USB port
arduinoSerial.timeout = None #Read calls should block
arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(0) + ";\n") #Turn Everything Off at beginning
class AccelerometerBind():
    def __init__(self):
        self.g = [0,0,1]
        try:
            self.ch = Spatial()
        except RuntimeError as e:
            print "FailAccel"
        self.stopped = True
        self.dynAccel = [0,0,0]
        self.previousDynAccel = [0,0,0]
        self.velocity = [0,0,0]
        self.previousVelocity = [0,0,0]
        self.distance = [0,0,0]
        self.prevTime = time.time()
        self.initTime = time.time()
        self.q = [0,0,0,0]
        self.gyroOffset = [0,0,0]
        self.F = open("/home/pi/MLRobot/Data2/AccelRawDyn" + time.strftime("%e:%H:%M:%S", time.localtime(time.time())),"w");
        line = "##########Next Run######### Trial " + str(trialNumber) + "\nTime,Ax,Ay,Az,Gx,Gy,Gz, DynX, DynY, DynZ\n"
        self.F.write(line);
        self.ch.setOnSpatialDataHandler(self.AccelerationChangeHandler)
        self.ch.openWaitForAttachment(5000)
        self.ch.setDataInterval(int(samplingTime))
        self.ch.zeroGyro()
    def reset(self):
        global trialNumber
        line = "##########Next Run######### Trial " + str(trialNumber) + "\nAx,Ay,Az,Gx,Gy,Gz, DynX, DynY, DynZ\n"
        self.F.write(line);
        self.dynAccel = [0,0,0] #x,y,z
        self.previousDynAccel = [0,0,0]
        self.velocity = [0,0,0]
        self.previousVelocity = [0,0,0]
        self.distance = [0,0,0]
        self.ch.zeroGyro()
        self.prevTime = time.time()
    def AccelerationChangeHandler(self,e, acceleration, angularRate , magneticField, timestamp):
        accel =  [0,0,0]
        gyro = [0,0,0]
        accel[0] = acceleration[0]*9.81
        accel[1] = acceleration[1]*9.81
        accel[2] = acceleration[2]*9.81
        gyro[0] = -angularRate[0] * 3.14159265/180
        gyro[1] = -angularRate[1] * 3.14159265/180
        gyro[2] = -angularRate[2] * 3.14159265/180
        if self.stopped:
            normalVector = np.cross(self.g, accel)
            normalVector = normalVector/LA.norm(normalVector)
            theta = acos(np.dot(accel,self.g)/(LA.norm(accel)*LA.norm(self.g)))
            self.q = [cos(theta/2),normalVector[0]*sin(theta/2),normalVector[1]*sin(theta/2),normalVector[2]*sin(theta/2)]
            self.prevTime = time.time()
            self.initialTime = time.time()
            print self.q
        if not self.stopped:
            currentTime = time.time()
            deltaT = currentTime - self.prevTime
            self.prevTime = currentTime
            if not LA.norm(gyro) < 0.005:
                a = LA.norm(gyro)*deltaT
                v = gyro/LA.norm(gyro)
                q_update = [cos(a/2),v[0]*sin(a/2),v[1]*sin(a/2),v[2]*sin(a/2)]
                self.q = self.q_mult(q_update,self.q) # Get rid of gyro TODO
            self.dynAccel = self.qv_mult(self.q_conjugate(self.q),accel) # conjugate first to get reverse rotation
            self.velocity = np.add(self.velocity, (deltaT/2)*np.add(self.dynAccel,self.previousDynAccel))
            self.distance = np.add(self.distance, (deltaT/2)*np.add(self.velocity,self.previousVelocity))
            self.F.write(str(time.time() - self.initialTime) + "," + str(acceleration[0]) + "," + str(acceleration[1]) + "," + str(acceleration[2]) + "," + str(angularRate[0]) + "," + str(angularRate[1]) + "," +str(angularRate[2]) + "," + str(self.dynAccel[0])+","+str(self.dynAccel[1])+","+str(self.dynAccel[2]) +"\n");
            self.previousDynAccel = self.dynAccel
            self.previousVelocity = self.velocity
            print self.distance[0], self.distance[1]
    def qv_mult(self,q1, v1):
        q2 = [0,0,0,0]
        q2[0] = 0
        q2[1] = v1[0]
        q2[2] = v1[1]
        q2[3] = v1[2]
        return self.q_mult(self.q_mult(q1, q2), self.q_conjugate(q1))[1:]
    def q_conjugate(self,q):
        w, x, y, z = q
        return (w, -x, -y, -z)
    def q_mult(self,q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return w, x, y, z
    def stop(self):
        self.stopped = True
        self.ch.setDataInterval(1000)
    def start(self):
        self.prevTime = time.time()
        self.dynAccel = [0,0,0] #x,y,z
        self.ch.zeroGyro()
        self.ch.setDataInterval(int(samplingTime))
        #sleep(2.05) # wait for gyro #TODO
        self.stopped = False
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
        self.accelIntX = 0
        self.accelIntY = 0
        self.displacementX = 0
        self.displacementY = 0
        self.yaw = 0
        self.heading = 0
    def update(self, Voltage, Ex,Ey, x1,y1,AccelIntX,AccelIntY):
        self.RightEncoder = rightMotorCount
        self.LeftEncoder = leftMotorCount
        self.Voltage = Voltage
        self.Ex = Ex
        self.Ey = Ey
        self.x1 = x1
        self.y1 = y1
        self.accelIntX = AccelIntX
        self.accelIntY = AccelIntY
class UserInputData():
    def __init__(self, myTime, leftPower, rightPower):
        self.mytime = myTime
        self.leftPower = leftPower
        self.rightPower = rightPower
inputData = UserInputData(0,-1,-1) #initial values
output = OutputData(0) #initial values
class EncoderIntegration(threading.Thread):
    global rightMotorCount, leftMotorCount, trialNumber
    def __init__(self):
        global rightMotorCount, leftMotorCount, trialNumber
        threading.Thread.__init__(self)
        self.prevREncoder = 0
        self.prevLEncoder = 0
        self.prevHeading = 0
        self.x = 0;
        self.y = 0
        self.velocityx = 0
        self.velocityy = 0
        self.heading = 0
        self.deltaL = 0
        self.deltaR = 0
        self.running = False
        self.initialTime = time.time()
        self.F = open("/home/pi/MLRobot/Data2/EncoderRawIntegrated" + time.strftime("%e:%H:%M:%S", time.localtime(time.time())),"w");
    def run(self):
        global leftMotorCount, rightMotorCount
        self.initialTime = prevtime = time.time()
        x = []
        y = []
        while True:
            if not self.running:
                sleep(0.1)
            if self.running:
                _in = arduinoSerial.readline() #block and read newest (very fast ideally)
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
                dy =  distance * cos(self.prevHeading+deltaTheta/2)
                self.x = self.x + dx
                self.y = self.y + dy
                self.velocityx = dx/dt
                self.velocityy = dy/dt
                x.append(self.x)
                y.append(self.y)
                line = str(time.time()-self.initialTime) + "," + str(rightMotorCount) + "," + str(leftMotorCount) + "," + str(self.velocityx) + "," + str(self.velocityy) + "," + str(self.x) + "," + str(self.y) + "\n"
                self.F.write(line);
                #print rightMotorCount, leftMotorCount
                #print "Encoder Integrating"
        #plt.plot(y,x,'bo')
        #plt.axis([0,max(y),max(abs(max(x)),abs(min(x))),-max(abs(max(x)),abs(min(x)))])
        #plt.savefig(os.path.join("/home/pi/MLRobot/Data", time.strftime("%e:%H:%M", time.localtime(time.time()))+"graph.png"),bbox_inches='tight',dpi= 300)
        #plt.close()
    def reset(self):
        arduinoSerial.timeout = 0.01
        self.prevREncoder = 0
        self.prevLEncoder = 0
        self.prevHeading = 0
        self.x = 0;
        self.y = 0
        self.velocityx = 0
        self.velocityy = 0
        self.heading = 0
        self.deltaL = 0
        self.deltaR = 0
        self.running = False
        self.initialTime = time.time()
    def startIntegrating(self):
        arduinoSerial.timeout = 0.01
        line = "##########Next Run######### Trial " + str(trialNumber) + "\nRightEncoder,LeftEncoder,VeclocityX,VelocityY,DistanceX,DistanceY,Time\n"
        self.F.write(line);
        self.running = True
    def stop(self):
        global leftMotorCount, rightMotorCount
        self.running = False
        #self.F.close()
        arduinoSerial.timeout = None
def wifiSocketRelayArduino(name) :
    global inputData
    data = dataSocket.recv(64)
    while not data:
        #arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(0) + ";\n")
        pass
    print data
    d = data.split("\n")[0].split(";")
    inputData.mytime = float(d[0])
    inputData.leftPower = float(d[1])
    inputData.rightPower = float(d[2])
def kalman(observed, prevState,prevCov,stateTrans,controlMat, controlVec, H ,dynNoiseCov, obsNoiseCov):
    x_predict = np.add(np.matmul(stateTrans, prevState), np.matmul(controlMat,controlVec))
    p_predict = np.add(np.matmul(np.matmul(stateTrans,prevCov),np.transpose(stateTrans)),np.matmul(np.matmul(controlMat,dynNoiseCov),np.transpose(controlMat)))
    yresid = np.subtract(observed,np.matmul(H,x_predict))
    S_k = np.add(obsNoiseCov,np.matmul(np.matmul(H,p_predict),np.transpose(H)))
    K = np.matmul(np.matmul(p_predict,np.transpose(H)),inv(S_k))
    x_update = np.add(x_predict, np.matmul(K,yresid))
    p_update = np.subtract(p_predict,np.matmul(np.matmul(K,H),p_predict))
    return (x_update, p_update)
AccelBind = AccelerometerBind()
E = EncoderIntegration()
E.start()
AccelBind.ch.setDataInterval(int(1000))
kalmFile = open("/home/pi/MLRobot/Data2/KalmanFilterRealTime" + time.strftime("%e:%H:%M:%S", time.localtime(time.time())),"w");
logger =  open("/home/pi/MLRobot/Data2/FullLogger" + time.strftime("%e:%H:%M:%S", time.localtime(time.time())),"w");

while True:
    line = "##########Next Run######### Trial " + str(trialNumber) + "\nTime,KalmanX,KalmanY,KalmanVX,KalmanVY\n"
    kalmFile.write(line);
    s = str(output.Voltage)+";"+str(output.Ex)+";"+str(output.Ey)+";"+str(output.accelIntX)+";"+str(output.accelIntY) +";"+str(output.x1)+";"+str(output.y1) +";\n" #create data string
    line = "##########Next Run######### Trial " + str(trialNumber) + "\nTime,KalmanX,KalmanY,AccelDispX, AccelDispY,EncoderX, EncoderY, KalmanVX,KalmanVY, AccelVX, AccelVY, EncoderVX, EncoderVY\n"
    logger.write(line)
    AccelBind.ch.setDataInterval(int(1000))
    try:
        wifiSocketRelayArduino("SocketArduinoComm") #gets and saves input Data from socket to arduino
    except socket.error as e:
        AccelBind.ch.close()
        print "Closed"
        raise
    AccelBind.start()
    arduinoSerial.timeout = None
    arduinoSerial.write(str(inputData.leftPower) + ";" + str(inputData.rightPower) + ";" + str(inputData.mytime) + ";\n")
    _in = arduinoSerial.readline()
    E.startIntegrating()
    print _in
    try:
        voltage = float(_in.rstrip("\n"))
    except ValueError:
        voltage = 0

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
    initTime = time.time()

    while (time.time()-initTime)*1000<(inputData.mytime) or not abs(E.velocityy) < 0.001 : #wait half second extra
        dt = time.time()-prevtime
        prevtime = time.time()
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
        raw = np.asarray([[AccelBind.dynAccel[0]],
                          [AccelBind.dynAccel[1]]
                        ])
        u_k = np.asarray([raw[0],               #control vector, known external influence
                          raw[1],
                          raw[0],
                          raw[1]
                        ])
        y = np.asarray([[E.x], #x from encoder
                        [E.y],  #y from encoder
                        [E.velocityx],
                        [E.velocityy]
                      ])
        H_k = np.asarray([[1, 0, 0, 0],
                          [0, 1, 0, 0],     #observation/state transformation matrix
                          [0, 0, 1, 0],
                          [0, 0, 0, 1],
                        ])
        A = np.asarray(np.diag((0.003,0.003,0.003,0.003))) #dynamic noise from Accel
        R = np.asarray(np.diag((0.00007,0.00007,0.0002,0.0002))) #from encoder
        [x1,p1] = kalman(y,x_0,p_0,F_k,B_k, u_k,H_k,A,R)
        x_0 = x1
        p_0 = p1
        line = str(time.time()-initTime) + "," + str(np.ravel(x_0[0])[0]) + "," + str(np.ravel(x_0[1])[0]) + "," + str(np.ravel(x_0[2])[0]) + "," + str(np.ravel(x_0[3])[0]) + "\n"
        kalmFile.write(line)
        #print(line)
        line =  str(time.time()-initTime) + "," + str(np.ravel(x_0[0])[0]) + "," + str(np.ravel(x_0[1])[0]) + "," + str(AccelBind.distance[0]) + "," + str(AccelBind.distance[1]) + "," + str(E.x) + "," + str(E.y) + ","
        line = line + str(np.ravel(x_0[2])[0]) + "," + str(np.ravel(x_0[3])[0]) + "," + str(AccelBind.velocity[0]) + "," + str(AccelBind.velocity[1]) + "," + str(E.velocityx) + "," + str(E.velocityy) + "\n"
        logger.write(line)
        sleep(0.01)
    E.stop()
    AccelBind.stop()
    output.update(voltage,E.x,E.y,np.ravel(x_0[0])[0],np.ravel(x_0[1])[0],AccelBind.distance[0],AccelBind.distance[1])
    trialNumber = trialNumber + 1
    s = str(output.Voltage)+";"+str(output.Ex)+";"+str(output.Ey)+";"+str(output.accelIntX)+";"+str(output.accelIntY) +";"+str(output.x1)+";"+str(output.y1) +";\n" #create data string
    AccelBind.reset()
    E.reset()
    dataSocket.send(s)
    rightMotorCount = 0
    leftMotorCount = 0
    print "Stopped"
    sleep(1);
