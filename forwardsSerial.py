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

#tested down to here

class colour:
	
	def start_colour_ticker(self, numTicker):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "startcrticker" + str(numTicker)
		com.write(Uinput)
		com.close()
		
	def stop_colour_ticker(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "stopcolourticker"
		com.write(Uinput)
		com.close()
	
	def detect_colour_once(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "detectcolouronce"
		com.write(Uinput)
		com.close()
		
	def get_colour_string(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "getcolourstring"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def read_base_colour_sensor_values(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "readbasecolour"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
class display:
	
	def clear_display(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "cleardisplay"
		com.write(Uinput)
		com.close()
		
	def home(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "home"
		com.write(Uinput)
		com.close()
		
	def write_string(self, Text):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "writestring" + str(Text)
		com.write(Uinput)
		com.close()
		
	
class led:
	
	def set_leds(self, grnValue, redValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "setleds" + str(grnValues) + str(redValues)
		com.write(Uinput)
		com.close()
		
	def set_green_leds(self,grnValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "setgreenleds" + str(grnValue)
		com.write(Uinput)
		com.close()
		
	def set_red_leds(self,redValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "setgreenleds" + str(redValue)
		com.write(Uinput)
		com.close()
		
	def set_led(self, ledNum, colourValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "setled" + str(ledNum) + str(colourValue)
		com.write(Uinput)
		com.close()
		
	def set_base_led(self, state):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "setbaseled" + str(state)
		com.write(Uinput)
		com.close()
		
	def blink_leds(self, ledValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "blinkleds" + str(ledValue)
		com.write(Uinput)
		com.close()
		
	def set_center_led(self, state):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "setcenterled" + str(state)
		com.write(Uinput)
		com.close()
		
	def set_center_led_brightness(self, light):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "setCledB" + str(light)
		com.write(Uinput)
		com.close()
		
	def get_led_states(self):
		com = serial.Serial('/dev/ttyACM1',baudrate=9600)
		Uinput = "getledstate" #still to test
		com.write(Uinput)
		time.sleep(0.5)
		while True:
			output = com.readline()
			if not com.strip():
				outputComplete= outputComplete + output
			else:
				break
		
		com.close()
		return outputComplete
	
#class sound:
	
class sensors:

	def get_dc_voltage(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "getbattvolt"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_current(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "getcurr"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_temperature(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "gettemp"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_battery_voltage(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "getdcvolt"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def enable_ultrasonic_ticker(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "enablesonicticker"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def disable_ultrasonic_ticker(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "disablesonicticker"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def update_ultrasonic_measure(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "updatesonicmeasure"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def store_background_raw_ir_values(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "storebgrawir"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def store_background_raw_ir_values(self): #Mistake on the mbed
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "storeillumrawir"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def store_ir_values(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "storeirvalues"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_background_raw_ir_value(self, settingValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "getbgrawir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_illuminated_raw_ir_value(self, settingValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "getillumrawir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def calculate_side_ir_value(self, settingValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "calculatesideir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def read_illuminated_raw_ir_value(self, settingValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "readillumrawir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def store_illumnated_base_ir_values(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "storeillumbaseir"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def store_base_ir_values(self):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "storebaseir"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_background_base_ir_value(self, settingValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "getbgbaseir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_illuminated_base_ir_value(self, settingValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "getillumbaseir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def calculate_base_ir_value(self, settingValue):
		com = serial.Serial('/dev/ttyACM0',baudrate=9600)
		Uinput = "calculatebaseir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output



#Testing area

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
#moveClass.stop()

animClass= animations()
#animClass.vibrate()
#time.sleep(1)
#animClass.set_colour(2) #don't know if it works
#time.sleep(2)
#animClass.led_run1()
#time.sleep(3)

coloClass = colour()
#coloClass.start_colour_ticker(500) #No visible difference but probably works
#time.sleep(2)
#coloClass.stop_colour_ticker() #No visible difference but probably works
#time.sleep(1)
#print(coloClass.get_colour_string()) #Doesn't give a result for some reason


sensClass = sensors()
print(sensClass.get_dc_voltage()) #Does give a result
