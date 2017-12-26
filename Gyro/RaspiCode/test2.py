import RPi.GPIO as GPIO
import threading
from time import sleep
Enc_A = 20
Enc_B = 21
state = 0
Rotary_counter = 0

def init():
	GPIO.setwarnings(True)
	GPIO.setmode(GPIO.BCM)					# Use BCM mode
											# define the Encoder switch inputs
	GPIO.setup(Enc_A, GPIO.IN)
	GPIO.setup(Enc_B, GPIO.IN)
											# setup callback thread for the A and B encoder
											# use interrupts for all inputs
	GPIO.add_event_detect(Enc_A, GPIO.BOTH, callback=rotary_interrupt) 				# NO bouncetime
	GPIO.add_event_detect(Enc_B, GPIO.BOTH, callback=rotary_interrupt) 				# NO bouncetime
	return

def rotary_interrupt(A_or_B):
	global Rotary_counter, state
	s = state & 3
	if GPIO.input(Enc_A):
		 s = s|4
	if GPIO.input(Enc_B):
		 s = s|8
	if s == 0 or s == 5 or s == 10 or s == 15:
		pass
	elif s == 1 or s == 7 or s == 8 or s == 14:
		Rotary_counter = Rotary_counter + 1
	elif s == 2 or s == 4 or s == 11 or s == 13:
		Rotary_counter = Rotary_counter - 1
	elif s == 3 or s == 12:
		Rotary_counter = Rotary_counter + 2
	else:
		Rotary_counter = Rotary_counter - 2
	state = s >> 2
	return


# Main loop. Demonstrate reading, direction and speed of turning left/rignt
def main():
	global Rotary_counter
	init()
	#oldValue = 0
	while True:
		print Rotary_counter
		sleep(0.02)
		#oldValue = Rotary_counter
main()
