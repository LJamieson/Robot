#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import forwardsSerial
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    sensorClass=forwardsSerial.sensors()
    animClass=forwardsSerial.animations();
    colourClass=forwardsSerial.colour();
    ledClass=forwardsSerial.led();
    moveClass=forwardsSerial.motors()
    #moveClass.set_right_motor_speed(1)
    #moveClass.forward(0.5)
    #time.sleep(1)
    #animClass.vibrate()
    #time.sleep(1)
    #moveClass.brake_right_motor()
    #moveClass.brake()
    #time.sleep(1)
    #moveClass.brake_right_motor()
    #time.sleep(1)
    #moveClass.forward(0.5)
    #time.sleep(1)
    #animClass.set_colour(2)
    #time.sleep(1)
    #animClass.led_run1()
    #time.sleep(1)
    #moveClass.brake_right_motor()
    #time.sleep(1)
    #moveClass.forward(0.5)
    #time.sleep(1)
    #moveClass.brake_right_motor()
    #time.sleep(1)
    #moveClass.forward(0.5)
    #time.sleep(1)
    #moveClass.brake_right_motor()
    #time.sleep(1)
    
    sensorValues = "Direct Current Voltage: %s" % sensorClass.get_dc_voltage()
    time.sleep(0.15)
    sensorValues = sensorValues + "Current: %s" % sensorClass.get_current()
    time.sleep(0.15)
    sensorValues = sensorValues + "Temperature: %s" % sensorClass.get_temperature()
    time.sleep(0.15)
    sensorValues = sensorValues + "Battery Voltage: %s" % sensorClass.get_battery_voltage()
    time.sleep(0.15)
    sensorClass.enable_ultrasonic_ticker()
    time.sleep(1)
    sensorClass.store_ir_values()
    time.sleep(0.15)
    sensorClass.get_background_raw_ir_value(0)
    time.sleep(0.15)
    sensorClass.get_illuminated_raw_ir_value(0)
    #sensorValues = sensorValues + "IR from sensor 0: %s" % sensorClass.calculate_side_ir_value(0)
    time.sleep(0.15)
    sensorClass.disable_ultrasonic_ticker()
    
    #moveClass.forward(0.5)
    #time.sleep(1)
    #moveClass.brake_right_motor()
    #time.sleep(1)
    
    #sensorClass.enable_ultrasonic_ticker()
    #sensor.get_background_raw_ir_value(0)
    #sensor.get_illuminated_raw_ir_value(0)
    #% sensorClass.calculate_side_ir_value(0)
    
    #colourClass.start_colour_ticker(10)
    #colourClass.stop_colour_ticker()
    #moveClass.forward(0.5)
    #time.sleep(1)
    #moveClass.stop()
    #time.sleep(0.015)
    #ledClass=forwardsSerial.led();
    #ledClass.set_leds(255,0)
    time.sleep(0.015)
    #hello_str = "hello world %s" % colourClass.read_base_colour_sensor_values()
    time.sleep(0.015)
    rospy.loginfo(sensorValues)
    pub.publish(sensorValues)
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
