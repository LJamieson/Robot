import serial
import time
#Motor movement Class
class motors:
		
	#Moves the psi swarm forwards
	def forward(self, Move):
		#Set the serial connection
		com = serialConnect()
		#Write the speed of the motor and send it to the mbed
		Uinput = "forward" + str(Move)
		com.write(Uinput)
		#Close the serial connection
		com.close()
	
	#Moves the psi swarm backwards	
	def backward(self, Move):
		#Set the serial connection
		com = serialConnect()
		#Write the speed of the motor going backwards and send it to the mbed
		Uinput = "backward" + str(Move)
		com.write(Uinput)
		#Close the serial connection
		com.close()
	
	#Stops the wheels of psi swarm	
	def brake(self):
		#Set the serial connection
		com = serialConnect()
		#Send brake to the mbed
		Uinput = "brake"
		com.write(Uinput)
		#Close the serial connection
		com.close()
	
	
	#Stops the psi swarm	
	def stop(self):
		#Set the serial connection
		com = serialConnect()
		#Send stop to the mbed
		Uinput = "stop"
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Stops the left motor
	def brake_left_motor(self):
		#Set the serial connection
		com = serialConnect()
		#Send brakeleft to the mbed
		Uinput = "brakeleft"
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Stops the right motor
	def brake_right_motor(self):
		#Set the serial connection
		com = serialConnect()
		#Send brakeright to the mbed
		Uinput = "brakeright"
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Sets the left motor's speed
	def set_left_motor_speed(self, Move):
		#Set the serial connection
		com = serialConnect()
		#Send leftmotor and movement speed to the mbed
		Uinput = "leftmotor" + str(Move)
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Sets the right motor's speed
	def set_right_motor_speed(self, Move):
		#Set the serial connection
		com = serialConnect()
		#Send rightmotor and movement speed to the mbed
		Uinput = "rightmotor" + str(Move)
		com.write(Uinput)
		#Close the serial connection
		com.close()

#Animations class
class animations:
		
	#Activates the vibrate command
	def vibrate(self):
		#Set the serial connection
		com = serialConnect()
		#Send vibrate to the mbed
		Uinput = "vibrate"
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Activates led run
	def led_run1(self):
		#Set the serial connection
		com = serialConnect()
		#Send ledrun1 to the mbed
		Uinput = "ledrun1"
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Sets the colour of the leds
	def set_colour(self, numColour):
		#Set the serial connection
		com = serialConnect()
		#Sends setcolour plus the number repressenting the colour to the mbed
		Uinput = "setcolour" + str(numColour)
		com.write(Uinput)
		#Close the serial connection
		com.close()

#tested down to here

#colour class
class colour:
	
	#Activates colour ticker
	def start_colour_ticker(self, numTicker):
		#Set the serial connection
		com = serialConnect()
		#Sends startcrticker and the timer to the mbed
		Uinput = "startcrticker" + str(numTicker)
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	def stop_colour_ticker(self):
		com = serialConnect()
		Uinput = "stopcolourticker"
		com.write(Uinput)
		com.close()
	
	def detect_colour_once(self):
		com = serialConnect()
		Uinput = "detectcolouronce"
		com.write(Uinput)
		com.close()
		
	def get_colour_string(self):
		com = serialConnect()
		Uinput = "getcolourstring"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def read_base_colour_sensor_values(self):
		com = serialConnect()
		Uinput = "readbasecolour"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
class display:
	
	def clear_display(self):
		com = serialConnect()
		Uinput = "cleardisplay"
		com.write(Uinput)
		com.close()
		
	def home(self):
		com = serialConnect()
		Uinput = "home"
		com.write(Uinput)
		com.close()
		
	def write_string(self, Text):
		com = serialConnect()
		Uinput = "writestring" + str(Text)
		com.write(Uinput)
		com.close()
		
	
class led:
	
	def set_leds(self, grnValue, redValue):
		com = serialConnect()
		Uinput = "setleds" + str(grnValue) + str(redValue)
		com.write(Uinput)
		com.close()
		
	def set_green_leds(self,grnValue):
		com = serialConnect()
		Uinput = "setgreenleds" + str(grnValue)
		com.write(Uinput)
		com.close()
		
	def set_red_leds(self,redValue):
		com = serialConnect()
		Uinput = "setredleds" + str(redValue)
		com.write(Uinput)
		com.close()
		
	def set_led(self, ledNum, colourValue):
		com = serialConnect()
		Uinput = "setled" + str(ledNum) + str(colourValue)
		com.write(Uinput)
		com.close()
		
	def set_base_led(self, state):
		com = serialConnect()
		Uinput = "setbaseled" + str(state)
		com.write(Uinput)
		com.close()
		
	def blink_leds(self, ledValue):
		com = serialConnect()
		Uinput = "blinkleds" + str(ledValue)
		com.write(Uinput)
		com.close()
		
	def set_center_led(self, state):
		com = serialConnect()
		Uinput = "setcenterled" + str(state)
		com.write(Uinput)
		com.close()
		
	def set_center_led_brightness(self, light):
		com = serialConnect()
		Uinput = "setCledB" + str(light)
		com.write(Uinput)
		com.close()
		
	def get_led_states(self):
		com = serialConnect()
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
		com = serialConnect()
		Uinput = "getbattvolt"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_current(self):
		com = serialConnect()
		Uinput = "getcurr"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_temperature(self):
		com = serialConnect()
		Uinput = "gettemp"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_battery_voltage(self):
		com = serialConnect()
		Uinput = "getdcvolt"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def enable_ultrasonic_ticker(self):
		com = serialConnect()
		Uinput = "enablesonicticker"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def disable_ultrasonic_ticker(self):
		com = serialConnect()
		Uinput = "disablesonicticker"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def update_ultrasonic_measure(self):
		com = serialConnect()
		Uinput = "updatesonicmeasure"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def store_background_raw_ir_values(self):
		com = serialConnect()
		Uinput = "storebgrawir"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def store_illumnated_raw_ir_values(self):
		com = serialConnect()
		Uinput = "storeillumrawir"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def store_ir_values(self):
		com = serialConnect()
		Uinput = "storeirvalues"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_background_raw_ir_value(self, settingValue):
		com = serialConnect()
		Uinput = "getbgrawir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_illuminated_raw_ir_value(self, settingValue):
		com = serialConnect()
		Uinput = "getillumrawir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def calculate_side_ir_value(self, settingValue):
		com = serialConnect()
		Uinput = "calculatesideir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def read_illuminated_raw_ir_value(self, settingValue):
		com = serialConnect()
		Uinput = "readillumrawir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def store_illumnated_base_ir_values(self):
		com = serialConnect()
		Uinput = "storeillumbaseir"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def store_base_ir_values(self):
		com = serialConnect()
		Uinput = "storebaseir"
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_background_base_ir_value(self, settingValue):
		com = serialConnect()
		Uinput = "getbgbaseir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def get_illuminated_base_ir_value(self, settingValue):
		com = serialConnect()
		Uinput = "getillumbaseir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
	def calculate_base_ir_value(self, settingValue):
		com = serialConnect()
		Uinput = "calculatebaseir" + str(settingValue)
		com.write(Uinput)
		time.sleep(0.5)
		output = com.readline()
		com.close()
		return output
		
# Connect the serial device
def serialConnect():
	return serial.Serial('/dev/ttyACM0',baudrate=115200)



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

#animClass= animations()
#animClass.vibrate()
#time.sleep(1)
#animClass.set_colour(2) #don't know if it works
#time.sleep(2)
#animClass.led_run1()
#time.sleep(3)

#coloClass = colour()
#coloClass.start_colour_ticker(500) #No visible difference but probably works
#time.sleep(2)
#coloClass.stop_colour_ticker() #No visible difference but probably works
#time.sleep(1)
#print(coloClass.get_colour_string()) #Doesn't give a result for some reason


#sensClass = sensors()
#print(sensClass.get_dc_voltage()) #Does give a result
