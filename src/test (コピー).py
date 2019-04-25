#! /usr/bin/env python

#MPU6050
#VCC -> PinNo.1
#SDA -> PinNo.3
#SCL -> PinNo.5
#GND -> PinNo.6

import rospy, tf, sys, os
import time
import subprocess
from sensor_msgs.msg import *
from geometry_msgs.msg import *

import errno, fcntl


if __name__ == "__main__":
	rospy.init_node("dmp_pub")	
	
	cmd = ["/home/raspberry-pi/PiBits/MPU6050-Pi-Demo/demo_dmp"]
	p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

	#flag = fcntl.fcntl(p.stdout.fileno(), fcntl.F_GETFL)
	#fcntl.fcntl(p.stdout.fileno(), fcntl.F_SETFL, flag | os.O_NONBLOCK)

	br = tf.TransformBroadcaster()

	while True:
		output = p.stdout.read()
		print output
		print "-"
		
		
	"""count = 0
	print "### START ###"
	for line in iter(p.stdout.readline,"\n"):
		try:
			#time.sleep(0.1)			
			#print count
			#count += 1		
			#if count > 15:			
			#sys.stdout.write(line)
			array = line.split()
			
			
			#print array[1]
			#print array[2]
			#print array[3]
			#print array[4]
			#print "-"
			x = float(array[1])
			y = float(array[2])
			z = float(array[3])
			w = float(array[4])
			print "x:%s, y:%s, z:%s, w:%s"%(str(x),str(y),str(z),str(w))
		
			br.sendTransform((1,0,0), (x,y,z,w), rospy.Time.now(), "mpu-6050", "map")
			#time.sleep(0.01)
			sys.stdout.flush()
			

		except KeyboardInterrupt:
			break
		
		except:
			rospy.logerr("!")"""
	
	print "### STOP ###"
	kill_cmd = ["pkill", "-x", "demo_dmp"]
	subprocess.call(kill_cmd)
	time.sleep(1)
	print "\n kill demo_dmp"

