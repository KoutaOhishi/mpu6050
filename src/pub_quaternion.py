#!/usr/bin/env python
import rospy
import tf
import time
import math

from modules import *
from geometry_msgs.msg import *

if __name__ == "__main__":
    rospy.init_node("mpu6050_quaternion_publisher")
    br = tf.TransformBroadcaster()

    # Sensor initialization
    mpu = mpu6050.MPU6050()
    mpu.dmpInitialize()
    mpu.setDMPEnabled(True)

    # get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize()

    while not rospy.is_shutdown():
        # Get INT_STATUS byte
        mpuIntStatus = mpu.getIntStatus()

        # check for DMP data ready interrupt (this should happen frequently)
        if mpuIntStatus >= 2:
            # get current FIFO count
            fifoCount = mpu.getFIFOCount()

            # check for overflow (this should never happen unless our code is too inefficient)
            # if fifoCount == 1024:
            # reset so we can continue cleanly
            # mpu.resetFIFO()
            #rospy.logwarn('FIFO overflow!')

            # wait for correct available data length, should be a VERY short wait
            fifoCount = mpu.getFIFOCount()
            while fifoCount < packetSize:
                fifoCount = mpu.getFIFOCount()

            result = mpu.getFIFOBytes(packetSize)
            q = mpu.dmpGetQuaternion(result)
            #g = mpu.dmpGetGravity(q)
            #ypr = mpu.dmpGetYawPitchRoll(q, g)
            rospy.loginfo(q)
            x = q["x"]
            y = q["y"]
            z = q["z"]
            w = q["w"]
            #br.sendTransform((1,0,0), (x,y,z,w), rospy.Time.now(), "mpu6050", "map")
            br.sendTransform(
                (0, -0.5, 0), (q["x"], q["y"], q["z"], q["w"]), rospy.Time.now(), "mpu6050", "map")
            mpu.resetFIFO()
            #print(ypr['yaw'] * 180 / math.pi),
            #print(ypr['pitch'] * 180 / math.pi),
            #print(ypr['roll'] * 180 / math.pi)

            # track FIFO count here in case there is > 1 packet available
            # (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize
