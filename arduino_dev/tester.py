import serial
import time

def measure(ser):
	ser.write(get_analog)
	time.sleep(1)
	while 1:
		data += ser.read(ser.inWaiting())
		if '\n' in buffer:
			return buffer


get_analog = "2/1/9/"
data = ""
ser = serial.Serial('/dev/ptyp5', 9600)

while 1:
	data = measure(ser)
	#if data != '/n':
	print "Analog reading: ", data
