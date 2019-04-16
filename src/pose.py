
#! /usr/bin/env python

import rospy, tf
from sensor_msgs.msg import *
from geometry_msgs.msg import *

pub = rospy.Publisher("imu_pose", PoseStamped, queue_size=1)
br = tf.TransformBroadcaster()

def callback(msg):
	pub_msg = PoseStamped()
	pub_msg.header = msg.header
	pub_msg.pose.orientation = msg.orientation
	pub.publish(pub_msg)

	br.sendTransform((0,0,0), msg.orientation, rospy.Time.now(), "mpu6050", "map")


if __name__ == "__main__":
	rospy.init_node("imu_pose")
	
	rospy.Subscriber("mpu6050_data", Imu, callback)
	rospy.spin()
