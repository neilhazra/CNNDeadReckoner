#!/usr/bin/env python3.4
import RPi.GPIO as GPIO
import serial
from time import sleep
import time

#SerialComm
arduinoSerial = serial.Serial('/dev/ttyACM0', 9600)	#Arduino at USB port
arduinoSerial.timeout = 1 #Read calls should block
arduinoSerial.write(str(-1) + ";" + str(-1) + ";" + str(0) + ";\n") #Turn Everything Off at beginning
#Encoders
GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP)
rightMotorCount = 0;
leftMotorCount = 0 ;

def RightMotorEncoder(channel):
	global rightMotorCount;
	#print "Right" + str(rightMotorCount)

	rightMotorCount = rightMotorCount + 0.1
def LeftMotorEncoder(channel):
	global leftMotorCount;
	#print "left" + str(rightMotorCount)
	leftMotorCount = leftMotorCount + 0.1

GPIO.add_event_detect(22, GPIO.BOTH, callback=LeftMotorEncoder, bouncetime=58)
GPIO.add_event_detect(10, GPIO.BOTH, callback=RightMotorEncoder, bouncetime=58)



#Get all data
initial = time.time()*1000
arduinoSerial.write(str(1) + ";" + str(1) + ";" + str(5000) + ";\n")
_in = arduinoSerial.readline()
print(_in)
voltage = float(_in.rstrip("\n"))
while time.time()*1000 - initial < 5000:
	sleep(0.01)
#print "stopped"
print rightMotorCount
print leftMotorCount
rightMotorCount = 0
leftMotorCount = 0
sleep(1);
