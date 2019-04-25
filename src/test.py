#! /usr/bin/env python

#MPU6050
#VCC -> PinNo.1
#SDA -> PinNo.3
#SCL -> PinNo.5
#GND -> PinNo.6

import rospy, tf, sys, os
import time, sys
import subprocess
from sensor_msgs.msg import *
from geometry_msgs.msg import *


if __name__ == "__main__":
	#rospy.init_node("dmp_pub")	
	
	cmd = ["/home/raspberry-pi/PiBits/MPU6050-Pi-Demo/demo_dmp"]
	p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
	#br = tf.TransformBroadcaster()

	while True:
		#time.sleep(0.1)
		try:
			out = p.stdout.readline(1)
			if out == "" and p.poll() != None:
				break
			if out != "":
				sys.stdout.write(out)
				sys.stdout.flush()
		except KeyboardInterrupt:
			break

	print "### STOP ###"
	kill_cmd = ["pkill", "-x", "demo_dmp"]
	subprocess.call(kill_cmd)
	time.sleep(1)
	print "\n kill demo_dmp"
