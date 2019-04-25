#! /usr/bin/env python

#MPU6050
#VCC -> PinNo.1
#SDA -> PinNo.3
#SCL -> PinNo.5
#GND -> PinNo.6

import smbus
import math
import time

import rospy, tf
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

def calcEuler(x,y,z):
	theta = math.atan( x / math.sqrt(y*y + z*z) )
	psi = math.atan( y / math.sqrt(x*x + z*z) )
	phi = math.atan( math.sqrt( x*x + y*y ) / z)

	#deg_theta = math.degrees( theta )
	#deg_psi = math.degrees( psi )
	#deg_phi = math.degrees( phi )

	#return [math.degrees( theta ), math.degrees( psi ), math.degrees( phi )]
	return [theta, psi, phi]

def euler2quaternion(x,y,z):
	q = tf.transformations.quaternion_from_euler(x,y,z)
	return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

if __name__ == "__main__":
	rospy.init_node("pub_imu")
	pub = rospy.Publisher("mpu6050_data", Imu, queue_size=1)
	br = tf.TransformBroadcaster()
		
	while True:
		try:
			gx, gy, gz = getGyro()
			ax, ay, az = getAccel()

			#roll = math.atan(ay/az)*57.324
			#pitch = math.atan(-ax/math.sqrt(ay*ay+az*az))*57.324
			roll, pitch, yaw = calcEuler(ax,ay,az)
			q = euler2quaternion(roll,pitch,yaw)
			#print ("{0:4.3f}, {0:4.3f}" .format(pitch, roll)) 
			
			"""imu = Imu()
			
			imu.orientation = q

			imu.angular_velocity.x = gx
			imu.angular_velocity.y = gy
			imu.angular_velocity.z = gz

			imu.linear_acceleration.x = ax
			imu.linear_acceleration.y = ay
			imu.linear_acceleration.z = az

			imu.header.frame_id = "mpu6050"

			pub.publish(imu)"""

			br.sendTransform((0,0.5,0), tf.transformations.quaternion_from_euler(roll,pitch,yaw), rospy.Time.now(), "mpu6050_", "map")

			#print "Roll:[%s]"%str(roll)
			#print "Pitch:[%s]"%str(pitch)
			#print "Yaw:[%s]"%str(yaw)
			
			print str(q)

			#print "Gyro : x[%s], y[%s], z[%s]"%(str(gx),str(gy),str(gz))
			#print "Accel : x[%s], y[%s], z[%s]"%(str(ax),str(ay),str(az))			
			
			#print "-----"	
		
		except KeyboardInterrupt:
			print "Ctrl + c" 
			break
		
		except Exception as e:
			#print "@@@"	
			#pass
			print str(e)
			#break



