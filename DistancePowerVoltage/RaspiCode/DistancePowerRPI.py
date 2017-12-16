#!/usr/bin/env python3.4
import RPi.GPIO as GPIO
import serial
from time import sleep
import time
from math import atan, atan2, sqrt
from thread import start_new_thread
import socket

#SsocketComm
dataSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
dataSocket.connect(( '10.0.0.102', 3800)) #rishi laptop
#dataSocket.connect(( '10.0.0.61', 3800)) #my laptop
#SerialComm
arduinoSerial = serial.Serial('/dev/ttyACM0', 9600)	#Arduino at USB port
arduinoSerial.timeout = None #Read calls should block
arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(0) + ";\n") #Turn Everything Off at beginning
#Encoders
GPIO.setmode(GPIO.BCM)
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
	wifiSocketRelayArduino("SocketArduinoComm") #gets and saves input Data from socket to arduino
	initial = time.time()*1000
	arduinoSerial.write(str(inputData.leftPower) + ";" + str(inputData.rightPower) + ";" + str(inputData.mytime) + ";\n")
	_in = arduinoSerial.readline()
	voltage = float(_in.rstrip("\n"))
	print _in
	print voltage
	while time.time()*1000 - initial < inputData.mytime:
		pass
		#print str(leftPower) + ";" + str(rightPower) + str((time.time()*1000-initial)/1000)
		#read gyro, encoder,etc()
		print "running"
	print "stopper"
	output.update(voltage)
	s = str(output.RightEncoder) + ";" + str(output.LeftEncoder) + ";" + str(output.Voltage) + ";\n" #create data string
	dataSocket.send(s)
	rightMotorCount = 0
	leftMotorCount = 0
	print "stopped"
	sleep(1);
