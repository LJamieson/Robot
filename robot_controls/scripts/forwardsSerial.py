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
		
	#Stops colour ticker
	def stop_colour_ticker(self):
		#Set the serial connection
		com = serialConnect()
		#Sends stopcolourticker to the mbed
		Uinput = "stopcolourticker"
		com.write(Uinput)
		#Close the serial connection
		com.close()
	
	#Detects colour once
	def detect_colour_once(self):
		#Set the serial connection
		com = serialConnect()
		#Sends dectcolouronce to the mbed
		Uinput = "detectcolouronce"
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Gets the colour and returns it
	def get_colour_string(self):
		#Set the serial connection
		com = serialConnect()
		#Sends getcolourstring to mbed
		Uinput = "getcolourstring"
		com.write(Uinput)
		#Gives time brake to allow the mbed to process the command
		time.sleep(0.5)
		#Receives the output from mbed 
		output = com.readline()
		#Close the serial connection
		com.close()
		return output
		
	#reads the base colour sensor values
	def read_base_colour_sensor_values(self):
		#Set the serial connection
		com = serialConnect()
		#Sends readbasecolour to mbed
		Uinput = "readbasecolour"
		com.write(Uinput)
		#Gives time brake to allow the mbed to process the command
		time.sleep(0.5)
		#Receives the output from mbed
		output = com.readline()
		#Close the serial connection
		com.close()
		return output
		
#display class
class display:
	
	#clears the display
	def clear_display(self):
		#Set the serial connection
		com = serialConnect()
		#Sends cleardisplay to mbed
		Uinput = "cleardisplay"
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Sets display to home
	def home(self):
		#Set the serial connection
		com = serialConnect()
		#Sends home to mbed
		Uinput = "home"
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Writes a message to show on the display
	def write_string(self, Text):
		#Set the serial connection
		com = serialConnect()
		#Sends writestring and the test to the mbed
		Uinput = "writestring" + str(Text)
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
#LED Class	
class led:
	
	#Sets the led colour values
	def set_leds(self, grnValue, redValue):
		#Set the serial connection
		com = serialConnect()
		#Sends setleds, the green and red value to the mbed
		Uinput = "setleds" + str(grnValue) + str(redValue)
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Sets the green led value
	def set_green_leds(self,grnValue):
		#Set the serial connection
		com = serialConnect()
		#Sends setgreenleds and the green value to the mbed
		Uinput = "setgreenleds" + str(grnValue)
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Sets the red led value
	def set_red_leds(self,redValue):
		#Set the serial connection
		com = serialConnect()
		#Sends setredleds and the red value to the mbed
		Uinput = "setredleds" + str(redValue)
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Sets a specified led to a colour value
	def set_led(self, ledNum, colourValue):
		#Set the serial connection
		com = serialConnect()
		#Sends setled and led number and colour value to the mbed
		Uinput = "setled" + str(ledNum) + str(colourValue)
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Sets the state of the base led
	def set_base_led(self, state):
		#Set the serial connection
		com = serialConnect()
		#Sends setbaseled and the state to the mbed
		Uinput = "setbaseled" + str(state)
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Makes the led blink
	def blink_leds(self, ledValue):
		#Set the serial connection
		com = serialConnect()
		#Sends blinkleds and the led value to the mbed
		Uinput = "blinkleds" + str(ledValue)
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Sets the state of the center led
	def set_center_led(self, state):
		#Set the serial connection
		com = serialConnect()
		#Sends setcenterled and state value to the mbed
		Uinput = "setcenterled" + str(state)
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Sets the center led's brightness
	def set_center_led_brightness(self, light):
		#Set the serial connection
		com = serialConnect()
		#Sends setCledB and the light value to the mbed
		Uinput = "setCledB" + str(light)
		com.write(Uinput)
		#Close the serial connection
		com.close()
		
	#Gets the state of the led
	def get_led_states(self):
		#Set the serial connection
		com = serialConnect()
		#Sends getledstate and the light value to the mbed
		Uinput = "getledstate" #still to test
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#While it has not broken
		while True:
			#Read the output
			output = com.readline()
			#If not the end of the line
			if not com.strip():
				#Add the out put to a series of previous outputs
				outputComplete= outputComplete + output
			else:
				break
		
		#Close the serial connection
		com.close()
		return outputComplete
	
#Sensors class	
class sensors:

	#Gets the dc voltage
	def get_dc_voltage(self):
		#Set the serial connection
		com = serialConnect()
		#Sends getbattvolt to the mbed
		Uinput = "getbattvolt"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Gets the current
	def get_current(self):
		#Set the serial connection
		com = serialConnect()
		#Sends getcurr to the mbed
		Uinput = "getcurr"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Gets the temperature
	def get_temperature(self):
		#Set the serial connection
		com = serialConnect()
		#Sends gettemp to the mbed
		Uinput = "gettemp"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
	
	#Gets the battery voltage	
	def get_battery_voltage(self):
		#Set the serial connection
		com = serialConnect()
		#Sends getdcvolt to the mbed
		Uinput = "getdcvolt"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Enables the ultrasonic ticker
	def enable_ultrasonic_ticker(self):
		#Set the serial connection
		com = serialConnect()
		#Sends enablesonicticker to the mbed
		Uinput = "enablesonicticker"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Disables the ultrasonic ticker
	def disable_ultrasonic_ticker(self):
		#Set the serial connection
		com = serialConnect()
		#Sends disablesonicticker to the mbed
		Uinput = "disablesonicticker"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Updates the ultrasonic measurement and returns it
	def update_ultrasonic_measure(self):
		#Set the serial connection
		com = serialConnect()
		#Sends updatesonicmeasure to the mbed
		Uinput = "updatesonicmeasure"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Stores backgroung raw ir values
	def store_background_raw_ir_values(self):
		#Set the serial connection
		com = serialConnect()
		#Sends storebgrawir to the mbed
		Uinput = "storebgrawir"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
	
	#Stores illumnated raw ir values	
	def store_illumnated_raw_ir_values(self):
		#Set the serial connection
		com = serialConnect()
		#Sends storeillumrawir to the mbed
		Uinput = "storeillumrawir"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Stores the ir values
	def store_ir_values(self):
		#Set the serial connection
		com = serialConnect()
		#Sends storeirvalues to the mbed
		Uinput = "storeirvalues"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Get background raw ir values
	def get_background_raw_ir_value(self, settingValue):
		#Set the serial connection
		com = serialConnect()
		#Sends getbgrawir and setting value to the mbed
		Uinput = "getbgrawir" + str(settingValue)
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Get illuminated raw ir value
	def get_illuminated_raw_ir_value(self, settingValue):
		#Set the serial connection
		com = serialConnect()
		#Sends getillumrawir and setting value to the mbed
		Uinput = "getillumrawir" + str(settingValue)
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Calculates side ir values
	def calculate_side_ir_value(self, settingValue):
		#Set the serial connection
		com = serialConnect()
		#Sends calculatesideir and setting value to the mbed
		Uinput = "calculatesideir" + str(settingValue)
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Reads illuminated raw ir value
	def read_illuminated_raw_ir_value(self, settingValue):
		#Set the serial connection
		com = serialConnect()
		#Sends readillumrawir and setting value to the mbed
		Uinput = "readillumrawir" + str(settingValue)
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Stores illumnated base ir values
	def store_illumnated_base_ir_values(self):
		#Set the serial connection
		com = serialConnect()
		#Sends storeillumbaseir to the mbed
		Uinput = "storeillumbaseir"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Stores base ir values
	def store_base_ir_values(self):
		#Set the serial connection
		com = serialConnect()
		#Sends storebaseir to the mbed
		Uinput = "storebaseir"
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Gets background base ir value
	def get_background_base_ir_value(self, settingValue):
		#Set the serial connection
		com = serialConnect()
		#Sends getbgbaseir and setting value to the mbed
		Uinput = "getbgbaseir" + str(settingValue)
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Gets illuminated base ir value
	def get_illuminated_base_ir_value(self, settingValue):
		#Set the serial connection
		com = serialConnect()
		#Sends getillumbaseir and setting value to the mbed
		Uinput = "getillumbaseir" + str(settingValue)
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
		com.close()
		return output
		
	#Calculate base ir values
	def calculate_base_ir_value(self, settingValue):
		#Set the serial connection
		com = serialConnect()
		#Sends calculatebaseir and setting value to the mbed
		Uinput = "calculatebaseir" + str(settingValue)
		com.write(Uinput)
		#Gives a time brake so that the mbed can process
		time.sleep(0.5)
		#Reads output from mbed
		output = com.readline()
		#Close the output
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
