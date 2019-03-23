import serial
import time
##Python class(float x)
class motors:
	#def __init__(self):
		#self.Move = Move	
	def forward(self, Move):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "forward" + str(Move)
		com.write(Uinput)
		com.close()
		
	def backward(self, Move):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "backward" + str(Move)
		com.write(Uinput)
		com.close()
		
	def brake(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "brake"
		com.write(Uinput)
		com.close()
		
	def stop(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "stop"
		com.write(Uinput)
		com.close()
		
	def brake_left_motor(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "brakeleft"
		com.write(Uinput)
		com.close()
		
	def brake_right_motor(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "brakeright"
		com.write(Uinput)
		com.close()
		
	def set_left_motor_speed(self, Move):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "leftmotor" + str(Move)
		com.write(Uinput)
		com.close()
		
	def set_right_motor_speed(self, Move):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "rightmotor" + str(Move)
		com.write(Uinput)
		com.close()

class animations:
		
	def vibrate(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "vibrate"
		com.write(Uinput)
		com.close()
		
	def led_run1(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "ledrun1"
		com.write(Uinput)
		com.close()
		
	def set_colour(self, numColour):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "setcolour" + str(numColour)
		com.write(Uinput)
		com.close()

#class colour:
		
#class display:
	
#class led:
	
class sensors:

	def get_dc_voltage(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "getbattvolt"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		##Print it
		com.close()
		return output

##Open serial port
##Write forwards(x) to serial
##close serial


##time.sleep(0.5)
##com.readline()



#moveClass = motors()
#moveClass.forward(0.5)
#time.sleep(2)
#moveClass.backward(0.8)
#time.sleep(2)
#moveClass.brake()
#time.sleep(2)
#moveClass.set_left_motor_speed(1)
#time.sleep(1)
#moveClass.set_right_motor_speed(0.5)
#time.sleep(3)
#moveClass.brake()

#animClass= animations()
#animClass.vibrate()
#time.sleep(1)
#animClass.set_colour(3)
#time.sleep(2)
#animClass.led_run1()
#time.sleep(3)



