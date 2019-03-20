import serial
import time
##Python class(float x)
class movementCommands:
	#def __init__(self):
		#self.Move = Move
		
	def forwards(self, Move):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "forward" + str(Move)
		com.write(Uinput)
		com.close()
		
	def backwards(self, Move):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "backward" + str(Move)
		com.write(Uinput)
		com.close()
		
	def brake(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "brake"
		com.write(Uinput)
		com.close()

##Open serial port
##Write forwards(x) to serial
##close serial

##time.sleep(0.5)
##com.readline()

moveClass = movementCommands()
moveClass.forwards(0.5)
time.sleep(1)
#moveClass = movementCommands(0.2)
moveClass.backwards(0.8)
time.sleep(1)
moveClass.brake()
