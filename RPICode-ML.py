#!/usr/bin/env python3.4
#RPI/Linux/System Imports
import RPi.GPIO as GPIO
import serial
import os
import logging
import threading
from time import sleep
import time
#SocketCommunications
import socket
#Math Libraries
from math import atan, atan2, sqrt, sin, cos, radians, degrees, acos
import numpy.matlib
from numpy.linalg import inv
from numpy import linalg as LA
import numpy as np
#Plotting Libraries
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
#Accelerometer
from Phidget22.Devices.Spatial import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *
#ML Imports
from sklearn.ensemble import RandomForestRegressor
import pickle
#TRIAL NUMBER
trialNumber = 1
#Constants
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
#dataSocket.connect(( '10.0.0.163', 3800)) #Dad laptop
#read ML model from file
regrXEncModel = '/home/pi/MLRobot/EncX-model.sav'
regrYEncModel = '/home/pi/MLRobot/EncY-model.sav'
regrXAccModel = '/home/pi/MLRobot/AccX-model.sav'
regrYAccModel = '/home/pi/MLRobot/AccY-model.sav'
regrXEnc = pickle.load(open(regrXEncModel, 'rb'))
regrYEnc = pickle.load(open(regrYEncModel, 'rb'))
regrXAcc = pickle.load(open(regrXAccModel, 'rb'))
regrYAcc = pickle.load(open(regrYAccModel, 'rb'))
print(regrXEnc.feature_importances_)
print(regrYEnc.feature_importances_)
print(regrXAcc.feature_importances_)
print(regrYAcc.feature_importances_)
#variance used in Gauss-Markov theorem
varEstEncoderErrX = 0.034571167 #0.000161498
varEstEncoderErrY = 0.017482655 #0.00001313
varEstAccelErrX = 0.034706856 #0.000207678
varEstAccelErrY = 0.949754853 #0.00010461

#SerialComm
arduinoSerial = serial.Serial('/dev/ttyACM0', 9600) #Arduino at USB port
print("Serial Arduino")
arduinoSerial.timeout = None #Read calls should block
arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(-1) + ";\n") #Turn Everything Off at beginning
print("Wrote Init to Arduino")

def calculateGaussianDistance(encoderX,encoderY,accelDispX,accelDispY,deltaT):
    #Estimate Accelerometer value (ML)
    estAccX = regrXAcc.predict([[deltaT,accelDispX, accelDispY]])[0]
    estAccY = regrYAcc.predict([[deltaT,accelDispX, accelDispY]])[0]

    #Estimate Encoder value (ML)
    estEncX = regrXEnc.predict([[deltaT,encoderX, encoderY]])[0]
    estEncY = regrYEnc.predict([[encoderX, encoderY]])[0]

    #Gauss-Markov estimation
    gaussDispX = (estEncX*varEstAccelErrX + estAccX*varEstEncoderErrX)/(varEstEncoderErrX + varEstAccelErrX)
    gaussDispY = (estEncY*varEstAccelErrY + estAccY*varEstEncoderErrY)/(varEstEncoderErrY + varEstAccelErrY)

    return gaussDispX, gaussDispY

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
            #print self.q
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
            #print self.distance[0], self.distance[1]
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
inputData = UserInputData(-1,-1,-1) #initial values
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
                    rightMotorCount = int(d[0])
                    leftMotorCount = int(d[1])
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
                #p rint rightMotorCount, leftMotorCount
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
class PID():
    def __init__(self,integral,derivative,errorLast):
        self.integral = integral
        self.derivative = derivative
        self.errorLast = errorLast
        self.Kp=0.0005
        self.Ki=0
        self.Kd=0.0
        self.deadBand = 0.001
    def getControlledOutput (self,error, dt):
        self.errorLast = error
        if abs(error) <= self.deadBand:
            output = 0
        else:
            output = ((self.Kp * error) + (self.Ki * self.integral)) + (self.Kd * self.derivative)

        self.integral += (error * dt);
        self.derivative = (error - self.errorLast) / dt;
        return output
AccelBind = AccelerometerBind()
E = EncoderIntegration()
E.start()
AccelBind.ch.setDataInterval(int(1000))
kalmFile = open("/home/pi/MLRobot/Data2/KalmanFilterRealTime" + time.strftime("%e:%H:%M:%S", time.localtime(time.time())),"w");
logger =  open("/home/pi/MLRobot/Data2/FullLogger" + time.strftime("%e:%H:%M:%S", time.localtime(time.time())),"w");
#Main Trial Loop
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
    #Change for PID control; set time=0
    baseLeftMotorPower = 0.35
    baseRightMotorPower = 0.65

    print "BeforeRead"
    #try to read arduino data
    arduinoSerial.timeout = 0.1
    while True:
        arduinoSerial.write(str(0) + ";" + str(0) + ";" + str(-1) + ";\n")
        print "Trying to read voltage"
        _in = arduinoSerial.readline()
        print _in
        try:
            voltage = float(_in.rstrip("\n"))
            print "Voltage Read"
            break
        except ValueError:
            voltage = -1
    arduinoSerial.timeout = None
    arduinoSerial.write(str(0) + ";" + str(0) + ";" + str(10) + ";\n")
    E.startIntegrating()
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

    #initialize
    prevEncDispX = prevEncDispY = 0
    prevAccDispX = prevAccDispY = 0
    gaussX = gaussY = 0
    totalDispX = totalDispY = 0

    pid = PID(0.0,0.0,0.0)
    print 'myTime:' + str(inputData.mytime)

    arduinoSerial.write(str(baseLeftMotorPower) + ";" + str(baseRightMotorPower) + ";" + str(100) + ";\n")
    #while (time.time()-initTime)*1000<(inputData.mytime) or not abs(E.velocityy) < 0.001 : #wait half second extra
    #To do: Let the robot run until target is reached; check (initialDistanceToTarget - gaussY) <= 0.001 (1 mm)
    while (time.time()-initTime)*1000 < (inputData.mytime):
        dt = time.time()-prevtime
        print 'in second while loop'
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

        #start of ML
        #distance travelled in last cycle
        deltaEncDispX = E.x - prevEncDispX
        deltaEncDispY = E.y - prevEncDispY
        deltaAccDispX = AccelBind.dynAccel[0] - prevAccDispX
        deltaAccDispY = AccelBind.dynAccel[1] - prevAccDispY
        print 'Acc.x,deltaAccDispX:' + str(AccelBind.dynAccel[0]) + ',' + str(deltaAccDispX)
        #store past cycle data
        prevEncDispX = E.x
        prevEncDispY = E.y
        prevAccDispX = AccelBind.dynAccel[0]
        prevAccDispY = AccelBind.dynAccel[1]
        # Use ML model to estimate the distance; convert to cm
        [gaussX,gaussY] = calculateGaussianDistance(deltaEncDispX*100,deltaEncDispY*100,deltaAccDispX*100,deltaAccDispY*100,dt)
        #print 'gaussX/gaussY:' + str(gaussX) +'/' + str(gaussY)
        #calculate total displacement since start
        totalDispX += deltaEncDispX
        totalDispY += deltaEncDispY
        #print 'totalDispX, totalDispY:' + str(totalDispX) + ',' + str(totalDispY)
        # Use PID here
        pidOut = pid.getControlledOutput(leftMotorCount-rightMotorCount,dt)

        #Error is negative => facing the target, the robot is going towards right, so apply more power to the right motor
        leftMotorPower  = baseLeftMotorPower - pidOut #*(1 - pidOut*0.5/(deltaEncDispY+0.0001))
        rightMotorPower = baseRightMotorPower + pidOut#*(1 + pidOut*0.5/(deltaEncDispY+0.0001))

        #print 'E.x/E.y:' + str(E.x) + '/' + str(E.y)
        print 'Encoder Left/Right:' + str(leftMotorCount) + '/' + str(rightMotorCount)
        print 'PID Out:' + str(pidOut)
        print 'leftPower/rightPower:' + str(leftMotorPower) +  '/' + str(rightMotorPower)

        arduinoSerial.write(str(rightMotorPower) + ";" + str(leftMotorPower) + ";" + str(100) + ";\n")
        #end of PID control

        y = np.asarray([[gaussX/100], #x from Gauss-Markov, convert to meter
                        [gaussY/100],  #y from Gauss-Markov, convert to meter
                        [E.velocityx],
                        [E.velocityy]
                      ])
        H_k = np.asarray([[1, 0, 0, 0],
                          [0, 1, 0, 0],     #observation/state transformation matrix
                          [0, 0, 1, 0],
                          [0, 0, 0, 1],
                        ])

        R = np.asarray(np.diag((0.00016*dt,0.00001*dt,0.00016,0.00001))) #from encoder

        A = np.asarray(np.diag((0.00000754,0.00000754,0.00000754,0.00000754))) #dynamic noise from Accel
        [x1,p1] = kalman(y,x_0,p_0,F_k,B_k, u_k,H_k,A,R)
        x_0 = x1
        p_0 = p1
        line = str(time.time()-initTime) + "," + str(np.ravel(x_0[0])[0]) + "," + str(np.ravel(x_0[1])[0]) + "," + str(np.ravel(x_0[2])[0]) + "," + str(np.ravel(x_0[3])[0]) + "\n"
        kalmFile.write(line)
        #print(line)
        line =  str(time.time()-initTime) + "," + str(np.ravel(x_0[0])[0]) + "," + str(np.ravel(x_0[1])[0]) + "," + str(AccelBind.distance[0]) + "," + str(AccelBind.distance[1]) + "," + str(E.x) + "," + str(E.y) + ","
        line = line + str(np.ravel(x_0[2])[0]) + "," + str(np.ravel(x_0[3])[0]) + "," + str(AccelBind.velocity[0]) + "," + str(AccelBind.velocity[1]) + "," + str(E.velocityx) + "," + str(E.velocityy) + "\n"
        logger.write(line)
        sleep(0.05)
    arduinoSerial.write(str(0) + ";" + str(0) + ";" + str(10) + ";\n")
    sleep(0.2);
    E.stop()
    arduinoSerial.write(str(0) + ";" + str(0) + ";" + str(-1) + ";\n")
    AccelBind.stop()
    output.update(voltage,E.x,E.y,np.ravel(x_0[0])[0],np.ravel(x_0[1])[0],AccelBind.distance[0],AccelBind.distance[1])
    trialNumber = trialNumber + 1
    s = str(output.Voltage) + ";" + str(output.Ex) + ";" + str(output.Ey) + ";" + str(output.accelIntX) + ";" + str(output.accelIntY) +";" + str(gaussX/100) +";" + str(gaussY/100) + ";" + str(output.x1) + ";" + str(output.y1) + ";\n" #create data string
    AccelBind.reset()
    E.reset()
    dataSocket.send(s)
    rightMotorCount = 0
    leftMotorCount = 0
    print "Stopped"
    sleep(1);
