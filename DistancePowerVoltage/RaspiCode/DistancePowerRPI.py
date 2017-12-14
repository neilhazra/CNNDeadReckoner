#!/usr/bin/env python3.4
import RPi.GPIO as GPIO
import serial
from time import sleep
import time
from math import atan, atan2, sqrt
from thread import start_new_thread
import smbus
import socket
address = 0x68
dataSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#dataSocket.connect(( '10.0.0.102', 3800)) #rishi laptop
dataSocket.connect(( '10.0.0.61', 3800)) #my laptop

arduinoSerial = serial.Serial('/dev/ttyACM0', 9600)
arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(0) + ";\n")

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

mytime = 0
leftPower = -1
rightPower = -1

def wifiSocketRelayArduino(name) :
	global mytime,leftPower,rightPower
	data = dataSocket.recv(64)
	while not data:
		arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(0) + ";\n")
		pass
	print data
	d = data.split("\n")[0].split(";")
	mytime = float(d[0])
	leftPower = float(d[1])
	rightPower = float(d[2])

def wifiSocketRelayLaptop(name) :
	s = str(rightMotorCount) + ";" + str(leftMotorCount) + ";\n"
	dataSocket.send(s)
	#print s
def communicate(name):
	wifiSocketRelayArduino(name)
	#wifiSocketRelayLaptop(name)

while True:
	communicate("test")
	initial = time.time()*1000
	arduinoSerial.write(str(leftPower) + ";" + str(rightPower) + ";" + str(mytime) + ";\n")
	while time.time()*1000 - initial < mytime:
		#print str(leftPower) + ";" + str(rightPower) + str((time.time()*1000-initial)/1000)
		#read gyro, encoder,etc
		print arduinoSerial.readline()
	print "stopped"
	sleep(1);
