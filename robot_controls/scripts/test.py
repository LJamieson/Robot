import time
import forwardsSerial

moveClass = forwardsSerial.motors()
sensorClass = forwardsSerial.sensors()
moveClass.forward(0.5)
time.sleep(1)
moveClass.stop()
time.sleep(1)
print(sensorClass.get_temperature())
#moveClass.backward(0.8)
time.sleep(1)
#moveClass.brake()
#time.sleep(2)
#moveClass.set_left_motor_speed(1)
#time.sleep(1)
#moveClass.set_right_motor_speed(0.5)
#time.sleep(3)
moveClass.stop()
