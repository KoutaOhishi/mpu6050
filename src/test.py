#! /usr/bin/env python

#MPU6050
#VCC -> PinNo.1
#SDA -> PinNo.3
#SCL -> PinNo.5
#GND -> PinNo.6

import smbus
import math
import time

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *

DEV_ADDR = 0x68

ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47

PWR_MGMT_1 = 0x6b
PWR_MGMT_2 = 0x6c

bus = smbus.SMBus(1)
bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

def read_word(adr):
	high = bus.read_byte_data(DEV_ADDR, adr)
	low = bus.read_byte_data(DEV_ADDR, adr+1)
	val = (high << 8) + low
	return val

#Sensor data read
def read_word_sensor(adr):
	val = read_word(adr)
	if (val >= 0x8000):#minus
		return -((65535 - val) + 1)
	else:#plus
		return val

def dist(a, b):
	return math.sqrt((a*a)+(b+b))

def get_y_rotation(x,y,z):
	radians = math.atan2(x, dist(y,z))
	return -math.degrees(radians)

def get_x_rotation(x,y,z):
	radians = math.atan2(y, dist(x,z))
	return math.degrees(radians)



def get_temp():
	temp = read_word_sensor(TEMP_OUT)
	x = temp / 340 + 36.53 #formula by data_sheet
	return x

def getGyro():
	x = read_word_sensor(GYRO_XOUT)/131.0
	y = read_word_sensor(GYRO_YOUT)/131.0
	z = read_word_sensor(GYRO_ZOUT)/131.0
	return [x, y, z]

def getAccel():
	x =read_word_sensor(ACCEL_XOUT)/16384.0
	y =read_word_sensor(ACCEL_YOUT)/16384.0
	z =read_word_sensor(ACCEL_ZOUT)/16384.0
	return [x, y, z]

if __name__ == "__main__":
	#rospy.init_node("pub_imu")
	#pub = rospy.Publisher("mpu6050_data", Imu, queue_size=1)
	
	##Get GyroZoffset
	gx_0, gy_0, gz_0 = getGyro()
	time.sleep(1)
	gx_1, gy_1, gz_1 = getGyro()
	offx = gx_1 - gx_0
	offy = gy_1 - gy_0
	offz = gz_1 - gz_0
		
	while True:
		try:
			gx, gy, gz = getGyro()
			ax, ay, az = getAccel()

			roll = math.atan(ay/az)*57.324
			pitch = math.atan(-ax/math.sqrt(ay*ay+az*az))*57.324
			yaw = math.atan( ((gz-offz)*math.sin(roll)-(gy-offy)*math.cos(roll)) / ((gx-offx)*math.cos(pitch)+(gy-offy)*math.sin(pitch)*math.sin(roll)+(gz-offz)*math.sin(pitch)*math.cos(roll)) )*57.324

			#print ("{0:4.3f}, {0:4.3f}" .format(pitch, roll)) 
			
			imu = Imu()
			
			imu.angular_velocity.x = gx
			imu.angular_velocity.y = gy
			imu.angular_velocity.z = gz

			imu.linear_acceleration.x = ax
			imu.linear_acceleration.y = ay
			imu.linear_acceleration.z = az

			#pub.publish(imu)

			print "Roll:[%s]"%str(roll)
			print "Pitch:[%s]"%str(pitch)
			print "Yaw:[%s]"%str(yaw)

			#print "Gyro : x[%s], y[%s], z[%s]"%(str(gx),str(gy),str(gz))
			#print "Accel : x[%s], y[%s], z[%s]"%(str(ax),str(ay),str(az))			
			
			print "-----"	
		
		except KeyboardInterrupt:
			print "Ctrl + c" 
			break
		
		except Exception as e:
			#print "@@@"	
			#pass
			print str(e)
			break



