#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import tf

def odometry_callback(msg):

	br = tf.TrandformBroadcaster()

	position = msg.pose.position
	quaternion = msg.pose.orientation

	# Envio para esta funcao a posicao e orientacao pela odometria, o tempo e ele vai fazer
	# a transformacao do frame world para a tf.
	br.sendTransform((position.x, position.y, 0), orientation, rospy.Time.now(), 
		'base_link', 'odom')


def link():

	rospy.init_node('robot_tf_broadcaster')

	rospy.Subscriber('/RosAria/pose', Odometry, odometry_callback)

	rospy.spin()

if __name__ == "__main__":
	try:
		link()
	except rospy.ROSInterruptException:
		pass
