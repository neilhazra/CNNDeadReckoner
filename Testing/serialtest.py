import serial
from time import sleep
ser = serial.Serial('/dev/ttyACM0', 9600)

while True:
	ser.write("HI From Raspberry Pi")
	sleep(10);