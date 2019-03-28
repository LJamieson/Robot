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
    moveClass=forwardsSerial.motors()
    moveClass.forward(0.5)
    time.sleep(1)
    moveClass.brake_right_motor()
    time.sleep(1)
    moveClass.forward(0.5)
    time.sleep(1)
    moveClass.brake_right_motor()
    time.sleep(1)
    moveClass.forward(0.5)
    time.sleep(1)
    moveClass.brake_right_motor()
    time.sleep(1)
    moveClass.forward(0.5)
    time.sleep(1)
    moveClass.brake_right_motor()
    time.sleep(1)
    moveClass.forward(0.5)
    time.sleep(1)
    moveClass.brake_right_motor()
    time.sleep(1)
    moveClass.forward(0.5)
    time.sleep(1)
    moveClass.stop()
    time.sleep(0.015)
    ledClass=forwardsSerial.led();
    ledClass.set_leds(255,0)
    time.sleep(0.015)
    hello_str = "hello world %s" % sensorClass.get_dc_voltage()
    time.sleep(0.015)
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
