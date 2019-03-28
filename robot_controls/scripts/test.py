import time
import forwardsSerial

moveClass = motors()
moveClass.forward(0.5)
#time.sleep(2)
#moveClass.backward(0.8)
#time.sleep(2)
#moveClass.brake()
#time.sleep(2)
#moveClass.set_left_motor_speed(1)
#time.sleep(1)
#moveClass.set_right_motor_speed(0.5)
#time.sleep(3)
moveClass.stop()
