#!/usr/bin/env python3.4
import RPi.GPIO as GPIO 
import time
 
GPIO.setmode(GPIO.BCM)  
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)  

rightMotorCount = 0;
leftMotorCount = 0;


def RightMotorEncoder(channel):  
	global rightMotorCount;
	rightMotorCount += 0.05;
	print(rightMotorCount);  
def LeftMotorEncoder(channel):
	global leftMotorCount;
	leftMotorCount += 0.05;  
	print(leftMotorCount);
GPIO.add_event_detect(17, GPIO.BOTH, callback=RightMotorEncoder, bouncetime=3)  
GPIO.add_event_detect(27, GPIO.BOTH, callback=LeftMotorEncoder, bouncetime=3)  



while True:
	time.sleep(10000);

