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


GPIO.setmode(GPIO.BCM)

#Gyro/Accel
bno = BNO055.BNO055(serial_port='/dev/ttyS0', rst=18)
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')
a = [1, 0, 232, 255, 252, 255, 145, 1, 3, 255, 72, 0, 255, 255, 254, 255, 0, 0, 232, 3, 108, 3]
bno.set_calibration(a)
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))



#SsocketComm
dataSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#dataSocket.connect(( '10.0.0.102', 3800)) #rishi laptop
dataSocket.connect(( '10.0.0.61', 3800)) #my laptop
#SerialComm
arduinoSerial = serial.Serial('/dev/ttyACM0', 9600)	#Arduino at USB port
arduinoSerial.timeout = None #Read calls should block
arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(0) + ";\n") #Turn Everything Off at beginning

#Encoders
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
rightMotorCount = 0;
leftMotorCount = 0 ;
def RightMotorEncoder(channel):
	global rightMotorCount;
	rightMotorCount = rightMotorCount + 0.05;
def LeftMotorEncoder(channel):
	global leftMotorCount;
	leftMotorCount = leftMotorCount + 0.05;
GPIO.add_event_detect(17, GPIO.BOTH, callback=RightMotorEncoder, bouncetime=3)
GPIO.add_event_detect(27, GPIO.BOTH, callback=LeftMotorEncoder, bouncetime=3)


class OutputData():
	global rightMotorCount, leftMotorCount
	def __init__(self, Voltage): #add more stuff once gyro works
		global rightMotorCount, leftMotorCount
		self.RightEncoder = rightMotorCount
		self.LeftEncoder = leftMotorCount
		self.Voltage = Voltage
	def update(self, Voltage):
		self.RightEncoder = rightMotorCount
		self.LeftEncoder = leftMotorCount
		self.Voltage = Voltage
		#other stuff to update gyro,accel etc
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



while True:
	#reset()
	velocityY = 0
	velocityX = 0
	distanceY = 0
	distanceX = 0

	wifiSocketRelayArduino("SocketArduinoComm") #gets and saves input Data from socket to arduino
	initial = time.time()*1000
	arduinoSerial.write(str(inputData.leftPower) + ";" + str(inputData.rightPower) + ";" + str(inputData.mytime) + ";\n")
	_in = arduinoSerial.readline()
	voltage = float(_in.rstrip("\n"))
	print _in
	print voltage
	prevtime = time.time()
	while time.time()*1000 - initial < inputData.mytime:
		deltaT = time.time()-prevtime
		prevtime = time.time()
		heading, roll, pitch = bno.read_euler()
		accelx,accely,accelz = bno.read_linear_acceleration()
		sys, gyro, accel, mag = bno.get_calibration_status()
		print('Sys_cal={0} Gyro_cal={1} Accel_cal={2} Mag_cal={3}'.format(sys, gyro, accel, mag))
		velocityX = velocityX + accelx * deltaT
		velocityY =  velocityY + accely * deltaT
		distanceX = distanceX + velocityX * deltaT
		distanceY =  distanceY + velocityY * deltaT
		print str(velocityY) + " " + str(distanceY) + " " + str(accely)
		#print('Heading={0:0.2F} accelX={1:0.2F} accelY={2:0.2F}'.format(heading, accelx, accely))
		sleep(0.001)
	print "stopped"
	output.update(voltage)
	s = str(output.RightEncoder) + ";" + str(output.LeftEncoder) + ";" + str(output.Voltage) + ";\n" #create data string
	dataSocket.send(s)
	rightMotorCount = 0
	leftMotorCount = 0
	print "stopped"
	sleep(1);
