#!/usr/bin/env python
# coding:utf-8
import smbus
import math
import time
import rospy
import tf
from sensor_msgs.msg import *
from geometry_msgs.msg import *

# アドレスの設定
DEV_ADDR = 0x68
ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
PWR_MGMT_1 = 0x6b
PWR_MGMT_2 = 0x6c

bus = smbus.SMBus(1)
bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)


def read_word(adr):
    high = bus.read_byte_data(DEV_ADDR, adr)
    low = bus.read_byte_data(DEV_ADDR, adr+1)
    val = (high << 8) + low
    return val


def read_word_sensor(adr):
    val = read_word(adr)
    if (val >= 0x8000):  # minus
        return -((65535 - val) + 1)
    else:  # plus
        return val


def getAccel():
    x = read_word_sensor(ACCEL_XOUT)/16384.0
    y = read_word_sensor(ACCEL_YOUT)/16384.0
    z = read_word_sensor(ACCEL_ZOUT)/16384.0
    return [x, y, z]


def calcEuler(x, y, z):
    theta = math.atan(x / math.sqrt(y*y + z*z))
    psi = math.atan(y / math.sqrt(x*x + z*z))
    phi = math.atan(math.sqrt(x*x + y*y) / z)
    return [theta, psi, phi]


def euler2quaternion(x, y, z):
    q = tf.transformations.quaternion_from_euler(x, y, z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


if __name__ == "__main__":
    rospy.init_node("mpu6050")
    br = tf.TransformBroadcaster()

    while True:
        try:
            ax, ay, az = getAccel()
            roll, pitch, yaw = calcEuler(ax, ay, az)
            q = euler2quaternion(roll, pitch, yaw)  # オイラー角をクォータ二オンに変換
            br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(
                roll, pitch, yaw), rospy.Time.now(), "mpu6050", "map")
            # print "Roll:[%s]"%str(roll)
            # print "Pitch:[%s]"%str(pitch)
            # print "Yaw:[%s]"%str(yaw)
        except KeyboardInterrupt:
            print("Ctrl + c")
            break
        except Exception as e:
            print(str(e))
            # break
