#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import String

Routput = ""

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global Routput
    Routput = data.data

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pylistener', anonymous=True)
    global Routput
    com = serial.Serial('/dev/ttyACM0',baudrate=115200)
    print("Connected to serial")
    while not rospy.is_shutdown():
		rospy.Subscriber("chatter", String, callback)
		print(Routput)
		if (Routput == "w"):
			com.write('w')
			print("writing w")
		elif (Routput == "s"):
			com.write('s')
			print("writing s")
		elif (Routput == "a"):
			com.write('a')
			print("writing a")
		elif (Routput == "d"):
			com.write('d')
			print("writing d")
		
		Routput=""
    
    
    com.close()
    
    
	
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    listener()
