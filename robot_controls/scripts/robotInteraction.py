#!/usr/bin/python3
##import rospy

##from std_msgs.msg import String

##def talker():
    ##pub = rospy.Publisher('chatter', String, queue_size=10)
    ##rospy.init_node('talker', anonymous=True)
    ##rate = rospy.Rate(50) # 10hz
    ##while not rospy.is_shutdown():
import serial		
com = serial.Serial('/dev/ttyACM0',baudrate=9600)
Uinput = raw_input("Please enter a command e.g 'forward0.5' :  ")
com.write(Uinput)
com.close()		
		
		#if Uinput == 'w':
		#	com.write('w')
		#elif Uinput == 's':
		#	com.write('s')
		#elif Uinput == 'a':
		#	com.write('a')
		#elif Uinput == 'd':
		#	com.write('d')
		
		

		##hello_str = "Wrote 'w' at %s" % rospy.get_time()
		##rospy.loginfo(hello_str)
		##pub.publish(hello_str)
		##rate.sleep()

##if __name__ == '__main__':
   ## try:
   ##     talker()
   ## except rospy.ROSInterruptException:
##pass
