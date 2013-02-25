import serial
import time
import getch

ser = serial.Serial('/dev/ttyUSB0',115200, timeout=1)
inkey = getch._Getch()

while True:
	ch = inkey()
	
	if ch == '':
		continue		
	if ch == 'W':
		ser.write('$2,300,0,0\r')
		continue		
	if ch == 'w':
		ser.write('$2,200,0,0\r')
		print ser.readline()
		continue
	if ch == 's':
		ser.write('$2,-200,0,0\r')
		print ser.readline()
		continue
	if ch == 'd':
		ser.write('$2,0,0,200\r')
		print ser.readline()
		continue
	if ch == 'a':
		ser.write('$2,0,0,-200\r')
		print ser.readline()
		continue
	if ch == ' ':
		ser.write('$0\r')
		continue
	if ch == 'q':
		ser.write('$0\r')
		break



