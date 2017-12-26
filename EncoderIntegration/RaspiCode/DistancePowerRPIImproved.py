#!/usr/bin/env python3.4
import RPi.GPIO as GPIO
import serial
from time import sleep
import time
from math import atan, atan2, sqrt
import math
from thread import start_new_thread
import threading
import socket

width = 6.0;
circumference = 7.75;

#SsocketComm
dataSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#dataSocket.connect(( '10.0.0.102', 3800)) #rishi laptop
#dataSocket.connect(( '10.0.0.61', 3800)) #my laptop
dataSocket.connect(( '10.0.0.61', 3800)) #my laptop

#SerialComm
arduinoSerial = serial.Serial('/dev/ttyACM0', 9600)	#Arduino at USB port
arduinoSerial.timeout = None #Read calls should block
arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(0) + ";\n") #Turn Everything Off at beginning
#Encoders
GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP)
rightMotorCount = 0;
leftMotorCount = 0 ;
initialTime = time.time()*1000

def RightMotorEncoder(channel):
	global rightMotorCount;
	#print "Right" + str(rightMotorCount)
	print "right" + str(rightMotorCount) + time.time()*1000-initialTime
	rightMotorCount = rightMotorCount + 0.1
def LeftMotorEncoder(channel):
	global leftMotorCount;
	print "left" + str(leftMotorCount)
	leftMotorCount = leftMotorCount + 0.1

GPIO.add_event_detect(22, GPIO.FALLING, callback=LeftMotorEncoder, bouncetime = 55)
GPIO.add_event_detect(10, GPIO.FALLING, callback=RightMotorEncoder, bouncetime = 55)


class OutputData():
	global rightMotorCount, leftMotorCount
	def __init__(self, Voltage): #add more stuff once gyro works
		global rightMotorCount, leftMotorCount
		self.RightEncoder = rightMotorCount
		self.LeftEncoder = leftMotorCount
		self.x = 0;
		self.y = 0;
		self.Voltage = Voltage
	def update(self, Voltage,x, y,heading):
		global rightMotorCount, leftMotorCount
		self.RightEncoder = rightMotorCount
		self.LeftEncoder = leftMotorCount
		self.Voltage = Voltage
		self.x = x;
		self.y = y;
		self.heading = heading;
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


class EncoderIntegrationY(threading.Thread):
	global rightMotorCount, leftMotorCount
	def __init__(self):
		global rightMotorCount, leftMotorCount
		threading.Thread.__init__(self)
		#self.F = open("/home/pi/EncoderIntegrationData/EncoderTest" + time.strftime("%e:%H:%M", time.localtime(time.time())),"w");
		self.prevREncoder = 0
		self.prevLEncoder = 0
		self.prevHeading = 0
		#self.x = 0;
		self.y = 0;
		self.heading = 0;
		self.running = True
	def run(self):
		while self.running:
			deltaR = rightMotorCount/20.0 - self.prevREncoder
			deltaL = leftMotorCount/20.0 - self.prevLEncoder
			self.prevREncoder = rightMotorCount/20.0
			self.prevLEncoder = leftMotorCount/20.0
			distance = circumference*(deltaR + deltaL)/2  # divide by 2
			#self.heading = self.prevHeading + atan2(circumference*(deltaL-deltaR),width)
			self.heading = atan2(circumference*(deltaL-deltaR),width)

			self.prevHeading = self.heading
			#self.x = self.x + distance * math.sin(self.heading)
			self.y = self.y + distance * math.cos(self.heading)
			#self.F.write(str(deltaR) + "," + str(deltaL) + "\n");
			sleep(0.05)
	def stop(self):
		self.running = False
class EncoderIntegrationX(threading.Thread):
	global rightMotorCount, leftMotorCount
	def __init__(self):
		global rightMotorCount, leftMotorCount
		threading.Thread.__init__(self)
		#self.F = open("/home/pi/EncoderIntegrationData/EncoderTest" + time.strftime("%e:%H:%M", time.localtime(time.time())),"w");
		self.prevREncoder = 0
		self.prevLEncoder = 0
		self.prevHeading = 0
		self.x = 0;
		#self.y = 0;
		self.heading = 0;
		self.deltaL = 0
		self.deltaR = 0
		self.running = True
	def run(self):
		while self.running:
			self.deltaR = rightMotorCount - self.prevREncoder
			self.deltaL = leftMotorCount - self.prevLEncoder
			if math.fabs(self.deltaR-self.deltaL)>0.11:
				self.prevREncoder = rightMotorCount
				self.prevLEncoder = leftMotorCount
				distance = circumference*(self.deltaR + self.deltaL)/2  # divide by 2
				#self.heading = self.prevHeading + atan2(circumference*(deltaL-deltaR),width)
				self.heading = atan2(circumference*(self.deltaL-self.deltaR),width)
				self.prevHeading = self.heading
				self.x = self.x + distance * math.sin(self.heading)
				print str(self.deltaR) + "," + str(self.deltaL)
				#self.y = self.y + distance * math.cos(self.heading)
				#self.F.write(str(deltaR) + "," + str(deltaL) + "\n");
			sleep(0.05)
	def stop(self):
		self.running = False

while True:
	#Get all data
	wifiSocketRelayArduino("SocketArduinoComm") #gets and saves input Data from socket to arduino
	initial = time.time()*1000
	#start run
	arduinoSerial.write(str(inputData.leftPower) + ";" + str(inputData.rightPower) + ";" + str(inputData.mytime) + ";\n")
	#saveEncoderData
	Ey = EncoderIntegrationY()

	Ex = EncoderIntegrationX()
	#get data
	_in = arduinoSerial.readline()
	Ey.start()
	Ex.start()
	print(_in)
	voltage = float(_in.rstrip("\n"))
	F = open("/home/pi/EncoderIntegrationData/EncoderTest" + time.strftime("%e:%H:%M", time.localtime(time.time())),"w");
	while time.time()*1000 - initial < inputData.mytime:
		#print str(leftPower) + ";" + str(rightPower) + str((time.time()*1000-initial)/1000)
		#read gyro, encoder,etc()
		F.write(str(Ex.deltaR) + "," + str(Ex.deltaL) + "," + str(rightMotorCount) + "," + str(leftMotorCount) + "\n");
		sleep(0.01)
	#print "stopped"
	sleep(1)
	output.update(voltage,Ex.x,Ey.y, Ey.heading)
	Ey.stop()
	Ex.stop()
	Ey.join()
	Ex.join()
	F.close()
	s = str(output.RightEncoder) + ";" + str(output.LeftEncoder) + ";" + str(output.Voltage) + ";" + str(output.x) + ";" + str(output.y) + ";" + str(output.heading)+ ";\n" #create data string
	dataSocket.send(s)
	rightMotorCount = 0
	leftMotorCount = 0
	print "stopped"
	sleep(1);
